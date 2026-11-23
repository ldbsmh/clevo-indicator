/*
 ============================================================================
 Name        : clevo-indicator.c
 Author      : AqD <iiiaqd@gmail.com>
 Version     :
 Description : Ubuntu fan control indicator for Clevo laptops

 Based on http://www.association-apml.fr/upload/fanctrl.c by Jonas Diemer
 (diemer@gmx.de)

 ============================================================================

 TEST:
 gcc clevo-indicator.c -o clevo-indicator `pkg-config --cflags --libs ayatana-appindicator3-0.1` -lm
 sudo chown root clevo-indicator
 sudo chmod u+s clevo-indicator

 Run as effective uid = root, but uid = desktop user (in order to use indicator).

 ============================================================================
 Auto fan control algorithm:

 The algorithm is to replace the builtin auto fan-control algorithm in Clevo
 laptops which is apparently broken in recent models such as W350SSQ, where the
 fan doesn't get kicked until both of GPU and CPU are really hot (and GPU
 cannot be hot anymore thanks to nVIDIA's Maxwell chips). It's far more
 aggressive than the builtin algorithm in order to keep the temperatures below
 60°C all the time, for maximized performance with Intel turbo boost enabled.

 CPU fan and GPU fan are always kept in sync - both are set to the same duty
 at all times.

 ============================================================================
 */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <libayatana-appindicator/app-indicator.h>

/* libayatana-appindicator >= 0.5.x 把 IS_APP_INDICATOR 改名为 APP_IS_INDICATOR */
#ifndef APP_IS_INDICATOR
#define APP_IS_INDICATOR(obj) IS_APP_INDICATOR(obj)
#endif

#define NAME "clevo-indicator"

#define EC_SC 0x66
#define EC_DATA 0x62

#define IBF 1
#define OBF 0
#define EC_SC_READ_CMD 0x80

/* EC registers can be read by EC_SC_READ_CMD or /sys/kernel/debug/ec/ec0/io:
 *
 * 1. modprobe ec_sys
 * 2. od -Ax -t x1 /sys/kernel/debug/ec/ec0/io
 */

#define EC_REG_SIZE         0x100
#define EC_REG_CPU_TEMP     0x07
#define EC_REG_GPU_TEMP     0xCD
#define EC_REG_FAN_DUTY     0xCE
#define EC_REG_FAN_RPMS_HI  0xD0  /* CPU fan RPM high byte */
#define EC_REG_FAN_RPMS_LO  0xD1  /* CPU fan RPM low byte */
#define EC_REG_FAN2_RPMS_HI 0xD2  /* GPU fan RPM high byte - may vary by model */
#define EC_REG_FAN2_RPMS_LO 0xD3  /* GPU fan RPM low byte - may vary by model */

/* EC fan control commands:
 *   ec_io_do(0x99, 0x01, value) -> CPU fan
 *   ec_io_do(0x99, 0x02, value) -> GPU fan
 */
#define EC_CMD_FAN          0x99
#define EC_PORT_FAN_CPU     0x01
#define EC_PORT_FAN_GPU     0x02

#define MAX_FAN_RPM 6000.0  /* T1000 实测 GPU 风扇满速约 5940 RPM */

/* 降速平滑窗口：取最近 N 次读数的平均值。
 * 升速使用瞬时温度立即响应，降速使用平均值延迟响应，防止风扇抖动。*/
#define TEMP_SMOOTH_DOWN_COUNT 30  /* 30 次 × 200ms = 约 6 秒 */

typedef enum {
    NA = 0, AUTO = 1, MANUAL = 2, EPP = 3, CUSTOM = 4, OVERLAY = 5
} MenuItemType;

/* 风扇占空比上下限。本机实测（T1000，双风扇）：
 *   60%→3802 RPM / 50%→3384 / 40%→2933 / 30%→2359 / 25%→2061
 *   20%→1730 / 15%→1318 / 12%→1069 / 10%→880
 * 10% 仍稳定转动、未失速，但留出积灰与轴承磨损的余量，手动下限取 15%。*/
#define FAN_MANUAL_MIN 15
#define FAN_AUTO_MIN   18

/* CPU 电源偏好档位：本机实测只有 power 能压住全核睿频，
 * balance_power / balance_performance / performance 三者表现完全相同
 * （空闲时也锁 3.9GHz），因此菜单只提供两个有实际区别的选项。*/
#define EPP_POWER   0   /* energy_performance_preference = power */
#define EPP_DEFAULT 1   /* 系统默认 = balance_performance */

static void main_init_share(void);
static int main_ec_worker(void);
static void main_ui_worker(int argc, char** argv);
static void main_on_sigchld(int signum);
static void main_on_sigterm(int signum);
static int main_dump_fan(void);
static int main_test_fan(int duty_percentage);
static gboolean ui_update(gpointer user_data);
static void ui_command_set_fan(long fan_duty);
static void ui_command_custom(long ignored);
static void ui_command_overlay(long ignored);
static void ui_command_set_epp(long which);
static void ui_refresh_menuitems(void);
static void ui_on_menu_show(GtkWidget* widget, gpointer data);
static void ui_load_css(void);
static void ui_overlay_show(void);
static GtkWidget* ui_overlay_cell(const char* caption, GtkWidget** value_out,
        int value_width, const char* tail, GtkWidget** tail_out);
static void ui_overlay_update(void);
static void ui_overlay_config_save(int visible, int x, int y);
static int ui_overlay_config_load(int* visible, int* x, int* y);
static int sysfs_apply_epp(int which);
static int sysfs_query_epp(void);
static char* epp_config_path(void);
static void epp_config_save(int which);
static int epp_config_load(void);
static void ec_on_sigterm(int signum);
static int ec_init(void);
static int ec_auto_duty_adjust(int temp_up, int temp_down);
static int ec_query_cpu_temp(void);
static int ec_query_gpu_temp(void);
static int ec_query_fan_duty(void);
static int ec_query_fan_rpms(void);
static int ec_query_fan2_rpms(void);
static int ec_write_fan_duty(int duty_percentage);
static int ec_io_wait(const uint32_t port, const uint32_t flag,
        const char value);
static uint8_t ec_io_read(const uint32_t port);
static int ec_io_do(const uint32_t cmd, const uint32_t port,
        const uint8_t value);
static int calculate_fan_duty(int raw_duty);
static int calculate_fan_rpms(int raw_rpm_high, int raw_rpm_low);
static int check_proc_instances(const char* proc_name);
static void get_time_string(char* buffer, size_t max, const char* format);
static void signal_term(__sighandler_t handler);

static AppIndicator* indicator = NULL;

/* 菜单对象需要被悬浮窗复用（右键弹同一份菜单），因此提到全局 */
static GtkWidget* indicator_menu = NULL;

/* 自定义转速窗口（拖动滑块连续调节）。
 * 用独立窗口而不是菜单项：SNI/dbusmenu 只能渲染普通菜单项，
 * 放不了滑块控件。*/
static GtkWidget* custom_window = NULL;
static GtkWidget* custom_value_label = NULL;
static GtkWidget* custom_rpm_label = NULL;
static guint custom_apply_timer = 0;
static int custom_pending_duty = 0;

/* 温度 / 转速悬浮窗：无边框卡片，置顶、半透明、可拖动。
 * 菜单只能显示文字，看不到实时读数，所以单独做一块小面板。*/
static GtkWidget* overlay_window = NULL;
static GtkWidget* overlay_cpu_val = NULL;
static GtkWidget* overlay_gpu_val = NULL;
static GtkWidget* overlay_fan_mode = NULL;
static GtkWidget* overlay_duty_val = NULL;
static GtkWidget* overlay_rpm_val = NULL;
static gboolean overlay_dragging = FALSE;
static int overlay_drag_dx = 0;
static int overlay_drag_dy = 0;
static int overlay_last_x = -1;
static int overlay_last_y = -1;

struct {
    char label[256];
    GCallback callback;
    long option;
    MenuItemType type;
    GtkWidget* widget;

}static menuitems[] = {
        { "CPU 电源：省电（推荐）", G_CALLBACK(ui_command_set_epp), EPP_POWER,
                EPP, NULL },
        { "CPU 电源：保持系统默认", G_CALLBACK(ui_command_set_epp), EPP_DEFAULT,
                EPP, NULL },
        { "", NULL, 0L, NA, NULL },
        { "风扇：自动（按温度）", G_CALLBACK(ui_command_set_fan), 0, AUTO, NULL },
        { "", NULL, 0L, NA, NULL },
        { "风扇：20%（静音）", G_CALLBACK(ui_command_set_fan), 20, MANUAL, NULL },
        { "风扇：30%", G_CALLBACK(ui_command_set_fan), 30, MANUAL, NULL },
        { "风扇：40%", G_CALLBACK(ui_command_set_fan), 40, MANUAL, NULL },
        { "风扇：50%", G_CALLBACK(ui_command_set_fan), 50, MANUAL, NULL },
        { "风扇：70%", G_CALLBACK(ui_command_set_fan), 70, MANUAL, NULL },
        { "风扇：100%（全速）", G_CALLBACK(ui_command_set_fan), 100, MANUAL, NULL },
        { "风扇：自定义…", G_CALLBACK(ui_command_custom), 0L, CUSTOM, NULL },
        { "", NULL, 0L, NA, NULL },
        { "悬浮窗：显示温度与转速", G_CALLBACK(ui_command_overlay), 0L, OVERLAY,
                NULL }
};

static int menuitem_count = (sizeof(menuitems) / sizeof(menuitems[0]));

struct {
    volatile int exit;
    volatile int cpu_temp;
    volatile int gpu_temp;
    volatile int fan_duty;
    volatile int fan_rpms;   /* CPU fan RPM */
    volatile int fan2_rpms;  /* GPU fan RPM */
    volatile int auto_duty;
    volatile int auto_duty_val;
    volatile int manual_next_fan_duty;
    volatile int manual_prev_fan_duty;
    volatile int epp_state;    /* 当前 CPU 电源档位 EPP_POWER / EPP_DEFAULT */
    volatile int epp_request;  /* -1 = 无请求；否则待应用的档位 */
}static *share_info = NULL;

static pid_t parent_pid = 0;

int main(int argc, char* argv[]) {
    /* 必须在任何输出之前设置：worker 的 stdout 是 socket，默认全缓冲，
     * 日志会卡在缓冲区里导致 journal 看不到。*/
    setvbuf(stdout, NULL, _IOLBF, 0);
    printf("Simple fan control utility for Clevo laptops\n");
    if (check_proc_instances(NAME) > 1) {
        printf("Multiple running instances!\n");
        char* display = getenv("DISPLAY");
        if (display != NULL && strlen(display) > 0) {
            int desktop_uid = getuid();
            setuid(desktop_uid);
            //
            gtk_init(&argc, &argv);
            GtkWidget* dialog = gtk_message_dialog_new(NULL, 0,
                    GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE,
                    "Multiple running instances of %s!", NAME);
            gtk_dialog_run(GTK_DIALOG(dialog));
            gtk_widget_destroy(dialog);
        }
        return EXIT_FAILURE;
    }
    if (ec_init() != EXIT_SUCCESS) {
        printf("unable to control EC: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
    if (argc <= 1) {
        char* display = getenv("DISPLAY");
        if (display == NULL || strlen(display) == 0) {
            return main_dump_fan();
        } else {
            parent_pid = getpid();
            main_init_share();
            signal(SIGCHLD, &main_on_sigchld);
            signal_term(&main_on_sigterm);
            pid_t worker_pid = fork();
            if (worker_pid == 0) {
                signal(SIGCHLD, SIG_DFL);
                signal_term(&ec_on_sigterm);
                return main_ec_worker();
            } else if (worker_pid > 0) {
                main_ui_worker(argc, argv);
                share_info->exit = 1;
                waitpid(worker_pid, NULL, 0);
            } else {
                printf("unable to create worker: %s\n", strerror(errno));
                return EXIT_FAILURE;
            }
        }
    } else {
        if (argv[1][0] == '-') {
            printf(
                    "\n\
Usage: clevo-indicator [fan-duty-percentage]\n\
\n\
Dump/Control fan duty on Clevo laptops. Display indicator by default.\n\
\n\
Arguments:\n\
  [fan-duty-percentage]\t\tTarget fan duty in percentage, from 15 to 100\n\
  -?\t\t\t\tDisplay this help and exit\n\
\n\
Without arguments this program should attempt to display an indicator in\n\
the Ubuntu tray area for fan information display and control. The indicator\n\
requires this program to have setuid=root flag but run from the desktop user\n\
, because a root user is not allowed to display a desktop indicator while a\n\
non-root user is not allowed to control Clevo EC (Embedded Controller that's\n\
responsible of the fan). Fix permissions of this executable if it fails to\n\
run:\n\
    sudo chown root clevo-indicator\n\
    sudo chmod u+s  clevo-indicator\n\
\n\
Note any fan duty change should take 1-2 seconds to come into effect - you\n\
can verify by the fan speed displayed on indicator icon and also louder fan\n\
noise.\n\
\n\
In the indicator mode, this program would always attempt to load kernel\n\
module 'ec_sys', in order to query EC information from\n\
'/sys/kernel/debug/ec/ec0/io' instead of polling EC ports for readings,\n\
which may be more risky if interrupted or concurrently operated during the\n\
process.\n\
\n\
DO NOT MANIPULATE OR QUERY EC I/O PORTS WHILE THIS PROGRAM IS RUNNING.\n\
\n\
CPU fan and GPU fan are always synchronized to the same duty.\n\
\n");
            return main_dump_fan();
        } else {
            int val = atoi(argv[1]);
            if (val < FAN_MANUAL_MIN || val > 100) {
                printf("invalid fan duty %d!\n", val);
                return EXIT_FAILURE;
            }
            return main_test_fan(val);
        }
    }
    return EXIT_SUCCESS;
}

static void main_init_share(void) {
    void* shm = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_ANON | MAP_SHARED,
            -1, 0);
    share_info = shm;
    share_info->exit = 0;
    share_info->cpu_temp = 0;
    share_info->gpu_temp = 0;
    share_info->fan_duty = 0;
    share_info->fan_rpms = 0;
    share_info->fan2_rpms = 0;
    share_info->auto_duty = 1;
    share_info->auto_duty_val = 0;
    share_info->manual_next_fan_duty = 0;
    share_info->manual_prev_fan_duty = 0;
    share_info->epp_state = sysfs_query_epp();
    share_info->epp_request = -1;
}

static int main_ec_worker(void) {
    setuid(0);
    system("modprobe ec_sys");
    int epp_ticks = 0;
    int manual_thermal_override = 0;
    while (share_info->exit == 0) {
        // check parent
        if (parent_pid != 0 && kill(parent_pid, 0) == -1) {
            printf("worker on parent death\n");
            break;
        }
        // CPU 电源偏好：UI 只负责发请求，实际写 sysfs 必须由 root 完成
        int epp_req = share_info->epp_request;
        if (epp_req >= 0) {
            int applied = sysfs_apply_epp(epp_req);
            if (applied > 0) {
                printf("CPU 电源偏好 -> %s（%d 个核心）\n",
                        epp_req == EPP_POWER ? "power" : "balance_performance",
                        applied);
                share_info->epp_state = epp_req;
            } else {
                printf("写入 CPU 电源偏好失败（需要 root 权限）\n");
            }
            share_info->epp_request = -1;
        } else if (++epp_ticks >= 25) {
            /* 每 5 秒回读一次，防止被其它程序改掉后菜单显示过期 */
            epp_ticks = 0;
            share_info->epp_state = sysfs_query_epp();
        }
        // write EC
        int new_fan_duty = share_info->manual_next_fan_duty;
        if (new_fan_duty != 0
                && new_fan_duty != share_info->manual_prev_fan_duty) {
            ec_write_fan_duty(new_fan_duty);
            share_info->manual_prev_fan_duty = new_fan_duty;
        }
        // read EC
        int io_fd = open("/sys/kernel/debug/ec/ec0/io", O_RDONLY, 0);
        if (io_fd < 0) {
            printf("unable to read EC from sysfs: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        unsigned char buf[EC_REG_SIZE];
        ssize_t len = read(io_fd, buf, EC_REG_SIZE);
        switch (len) {
        case -1:
            printf("unable to read EC from sysfs: %s\n", strerror(errno));
            break;
        case 0x100:
            share_info->cpu_temp = buf[EC_REG_CPU_TEMP];
            /* 部分机型（如混合显卡模式的 T1000）EC 侧 GPU 二极管
             * 不上报，0xCD 恒为 0。非法值按 0 处理，避免自动模式
             * 被错误温度拉高转速，也避免界面显示假的温度。*/
            {
                int gpu = buf[EC_REG_GPU_TEMP];
                share_info->gpu_temp = (gpu > 0 && gpu < 110) ? gpu : 0;
            }
            share_info->fan_duty = calculate_fan_duty(buf[EC_REG_FAN_DUTY]);
            share_info->fan_rpms = calculate_fan_rpms(buf[EC_REG_FAN_RPMS_HI],
                    buf[EC_REG_FAN_RPMS_LO]);
            share_info->fan2_rpms = calculate_fan_rpms(buf[EC_REG_FAN2_RPMS_HI],
                    buf[EC_REG_FAN2_RPMS_LO]);
            break;
        default:
            printf("wrong EC size from sysfs: %ld\n", len);
        }
        close(io_fd);
        /* 手动档的安全网：允许把转速压到 15%，但温度顶到 93°C 时
         * 临时拉满，降到 80°C 以下再交还给用户设定的档位。
         * 防止误设低档后忘了改回去而烧机器。*/
        if (share_info->auto_duty == 1) {
            manual_thermal_override = 0;
        } else {
            int t = MAX(share_info->cpu_temp, share_info->gpu_temp);
            if (!manual_thermal_override && t >= 93
                    && share_info->fan_duty < 100) {
                char s_time[256];
                get_time_string(s_time, 256, "%m/%d %H:%M:%S");
                printf("%s 手动档 %d%% 但温度 %d°C 超限，临时全速\n", s_time,
                        share_info->manual_next_fan_duty, t);
                ec_write_fan_duty(100);
                manual_thermal_override = 1;
            } else if (manual_thermal_override && t <= 80) {
                int target = share_info->manual_next_fan_duty;
                if (target >= FAN_MANUAL_MIN) {
                    char s_time[256];
                    get_time_string(s_time, 256, "%m/%d %H:%M:%S");
                    printf("%s 温度回落到 %d°C，恢复手动档 %d%%\n", s_time, t,
                            target);
                    ec_write_fan_duty(target);
                }
                manual_thermal_override = 0;
            }
        }
        // auto EC
        if (share_info->auto_duty == 1) {
            /* 升速：使用瞬时温度，立即响应温度峰值 */
            int temp_up = MAX(share_info->cpu_temp, share_info->gpu_temp);

            /* 降速：使用长期平均温度，温度持续回落才降档，防止风扇抖动 */
            static int cpu_history[TEMP_SMOOTH_DOWN_COUNT];
            static int gpu_history[TEMP_SMOOTH_DOWN_COUNT];
            static int history_idx = 0;
            static int history_count = 0;
            cpu_history[history_idx] = share_info->cpu_temp;
            gpu_history[history_idx] = share_info->gpu_temp;
            history_idx = (history_idx + 1) % TEMP_SMOOTH_DOWN_COUNT;
            if (history_count < TEMP_SMOOTH_DOWN_COUNT) history_count++;
            int cpu_sum = 0, gpu_sum = 0;
            for (int i = 0; i < history_count; i++) {
                cpu_sum += cpu_history[i];
                gpu_sum += gpu_history[i];
            }
            int temp_down = MAX(cpu_sum / history_count, gpu_sum / history_count);

            int next_duty = ec_auto_duty_adjust(temp_up, temp_down);
            if (next_duty != 0 && next_duty != share_info->auto_duty_val) {
                char s_time[256];
                get_time_string(s_time, 256, "%m/%d %H:%M:%S");
                printf("%s CPU=%d°C, GPU=%d°C, avg=%d°C, auto fan duty to %d%%\n", s_time,
                        share_info->cpu_temp, share_info->gpu_temp, temp_down, next_duty);
                ec_write_fan_duty(next_duty);
                share_info->auto_duty_val = next_duty;
            }
        }
        //
        usleep(200 * 1000);
    }
    printf("worker quit\n");
    return EXIT_SUCCESS;
}

static void main_ui_worker(int argc, char** argv) {
    printf("Indicator...\n");
    int desktop_uid = getuid();
    setuid(desktop_uid);
    //
    gtk_init(&argc, &argv);
    ui_load_css();
    //
    indicator_menu = gtk_menu_new();
    for (int i = 0; i < menuitem_count; i++) {
        GtkWidget* item;
        if (strlen(menuitems[i].label) == 0) {
            item = gtk_separator_menu_item_new();
        } else {
            item = gtk_menu_item_new_with_label(menuitems[i].label);
            g_signal_connect_swapped(item, "activate",
                    G_CALLBACK(menuitems[i].callback),
                    (void* ) menuitems[i].option);
        }
        gtk_menu_shell_append(GTK_MENU_SHELL(indicator_menu), item);
        menuitems[i].widget = item;
    }
    gtk_widget_show_all(indicator_menu);
    /* 每次展开菜单前刷新勾选状态，避免显示过期 */
    g_signal_connect(indicator_menu, "show", G_CALLBACK(ui_on_menu_show), NULL);
    //
    indicator = app_indicator_new(NAME, "brasero",
            APP_INDICATOR_CATEGORY_HARDWARE);
    g_assert(APP_IS_INDICATOR(indicator));
    app_indicator_set_label(indicator, "Init..", "XX");
    app_indicator_set_status(indicator, APP_INDICATOR_STATUS_ATTENTION);
    app_indicator_set_ordering_index(indicator, -2);
    app_indicator_set_title(indicator, "Clevo");
    app_indicator_set_menu(indicator, GTK_MENU(indicator_menu));
    g_timeout_add(500, &ui_update, NULL);
    {
        /* 恢复上次选择的 CPU 电源档位 */
        int saved = epp_config_load();
        if (saved >= 0) {
            share_info->epp_request = saved;
            share_info->epp_state = saved;
        }
    }
    ui_refresh_menuitems();
    {
        /* 上次开着悬浮窗的话，开机自动恢复 */
        int vis = 0, ox = -1, oy = -1;
        if (ui_overlay_config_load(&vis, &ox, &oy) && vis)
            ui_overlay_show();
    }
    gtk_main();
    printf("main on UI quit\n");
}

static void main_on_sigchld(int signum) {
    printf("main on worker quit signal\n");
    exit(EXIT_SUCCESS);
}

static void main_on_sigterm(int signum) {
    printf("main on signal: %s\n", strsignal(signum));
    if (share_info != NULL)
        share_info->exit = 1;
    exit(EXIT_SUCCESS);
}

static int main_dump_fan(void) {
    int gpu_temp = ec_query_gpu_temp();
    printf("Dump fan information\n");
    printf("  CPU FAN Duty: %d%%\n", ec_query_fan_duty());
    printf("  CPU FAN RPMs: %d RPM\n", ec_query_fan_rpms());
    printf("  GPU FAN RPMs: %d RPM\n", ec_query_fan2_rpms());
    printf("  CPU Temp:     %d°C\n", ec_query_cpu_temp());
    if (gpu_temp > 0 && gpu_temp < 110)
        printf("  GPU Temp:     %d°C\n", gpu_temp);
    else
        printf("  GPU Temp:     N/A (EC 未上报此机型)\n");
    return EXIT_SUCCESS;
}

static int main_test_fan(int duty_percentage) {
    printf("Change fan duty to %d%% (CPU + GPU)\n", duty_percentage);
    ec_write_fan_duty(duty_percentage);
    printf("\n");
    main_dump_fan();
    return EXIT_SUCCESS;
}

static gboolean ui_update(gpointer user_data) {
    char label[256];
    if (share_info->gpu_temp > 0) {
        sprintf(label, "%d℃ %d℃", share_info->cpu_temp, share_info->gpu_temp);
    } else {
        sprintf(label, "%d℃ --", share_info->cpu_temp);
    }
    app_indicator_set_label(indicator, label, "XXXXXX");
    /* 悬浮窗跟随刷新：温度、档位、转速 */
    ui_overlay_update();
    /* 自定义窗口打开时同步显示实际占空比与转速 */
    if (custom_rpm_label != NULL) {
        char buf[128];
        snprintf(buf, sizeof(buf), "当前 %d%%  ·  %d RPM",
                share_info->fan_duty, share_info->fan_rpms);
        gtk_label_set_text(GTK_LABEL(custom_rpm_label), buf);
    }
    /* Use average RPM of CPU and GPU fans for icon animation */
    int avg_rpms = (share_info->fan_rpms + share_info->fan2_rpms) / 2;
    char icon_name[256];
    /* 若图标主题没有 brasero-disc-XX 动画图标，则回退到静态图标 */
    static int has_anim_icons = -1;
    if (has_anim_icons < 0) {
        GtkIconTheme* theme = gtk_icon_theme_get_default();
        has_anim_icons = gtk_icon_theme_has_icon(theme, "brasero-disc-50") ? 1 : 0;
    }
    double load = ((double) avg_rpms) / MAX_FAN_RPM * 100.0;
    double load_r = round(load / 5.0) * 5.0;
    if (has_anim_icons) {
        sprintf(icon_name, "brasero-disc-%02d", (int) load_r);
    } else {
        sprintf(icon_name, "brasero");
    }
    app_indicator_set_icon(indicator, icon_name);
    return G_SOURCE_CONTINUE;
}

static void ui_command_set_fan(long fan_duty) {
    int fan_duty_val = (int) fan_duty;
    if (fan_duty_val == 0) {
        printf("clicked on fan duty auto\n");
        share_info->auto_duty = 1;
        share_info->auto_duty_val = 0;
        share_info->manual_next_fan_duty = 0;
    } else {
        printf("clicked on fan duty: %d\n", fan_duty_val);
        share_info->auto_duty = 0;
        share_info->auto_duty_val = 0;
        share_info->manual_next_fan_duty = fan_duty_val;
    }
    ui_refresh_menuitems();
}

/* ---------- 自定义转速窗口 ---------- */

static gboolean ui_custom_apply(gpointer data) {
    custom_apply_timer = 0;
    if (share_info != NULL)
        ui_command_set_fan((long) custom_pending_duty);
    return G_SOURCE_REMOVE;
}

static void ui_custom_set_value_label(int v) {
    if (custom_value_label == NULL)
        return;
    char buf[128];
    snprintf(buf, sizeof(buf),
            "<span size='xx-large' weight='bold'>%d%%</span>", v);
    gtk_label_set_markup(GTK_LABEL(custom_value_label), buf);
}

static void ui_custom_changed(GtkRange* range, gpointer data) {
    int v = (int) gtk_range_get_value(range);
    custom_pending_duty = v;
    ui_custom_set_value_label(v);
    /* 拖动过程中不要每移动 1% 就写一次 EC：停止拖动 300ms 后才下发 */
    if (custom_apply_timer != 0)
        g_source_remove(custom_apply_timer);
    custom_apply_timer = g_timeout_add(300, ui_custom_apply, NULL);
}

static void ui_custom_destroy(GtkWidget* widget, gpointer data) {
    if (custom_apply_timer != 0) {
        g_source_remove(custom_apply_timer);
        custom_apply_timer = 0;
    }
    custom_window = NULL;
    custom_value_label = NULL;
    custom_rpm_label = NULL;
}

static void ui_command_custom(long ignored) {
    if (custom_window != NULL) {
        gtk_window_present(GTK_WINDOW(custom_window));
        return;
    }

    GtkWidget* win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(win), "风扇转速");
    gtk_window_set_keep_above(GTK_WINDOW(win), TRUE);
    gtk_window_set_position(GTK_WINDOW(win), GTK_WIN_POS_CENTER);
    gtk_window_set_resizable(GTK_WINDOW(win), FALSE);
    gtk_window_set_default_size(GTK_WINDOW(win), 400, -1);

    GtkWidget* box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_container_set_border_width(GTK_CONTAINER(box), 20);
    gtk_container_add(GTK_CONTAINER(win), box);

    GtkWidget* title = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(title),
            "<span size='large' weight='bold'>拖动滑块调节风扇</span>");
    gtk_box_pack_start(GTK_BOX(box), title, FALSE, FALSE, 0);

    custom_value_label = gtk_label_new(NULL);
    gtk_box_pack_start(GTK_BOX(box), custom_value_label, FALSE, FALSE, 0);

    GtkWidget* scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL,
            FAN_MANUAL_MIN, 100, 1);
    gtk_scale_set_draw_value(GTK_SCALE(scale), FALSE);
    gtk_range_set_increments(GTK_RANGE(scale), 1, 10);
    gtk_widget_set_size_request(scale, 340, -1);
    gtk_box_pack_start(GTK_BOX(box), scale, FALSE, FALSE, 6);

    GtkWidget* ends = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    char ends_left[64];
    snprintf(ends_left, sizeof(ends_left),
            "<span size='small'>%d%%  最静</span>", FAN_MANUAL_MIN);
    GtkWidget* l = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(l), ends_left);
    gtk_box_pack_start(GTK_BOX(ends), l, FALSE, FALSE, 0);
    GtkWidget* r = gtk_label_new("<span size='small'>100%%  全速</span>");
    gtk_label_set_markup(GTK_LABEL(r), "<span size='small'>100%  全速</span>");
    gtk_box_pack_end(GTK_BOX(ends), r, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(box), ends, FALSE, FALSE, 0);

    custom_rpm_label = gtk_label_new("当前 --");
    gtk_box_pack_start(GTK_BOX(box), custom_rpm_label, FALSE, FALSE, 0);

    GtkWidget* hint = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(hint),
            "<span size='small' color='#888888'>"
            "拖动即切换到手动档，停止拖动后 0.3 秒生效</span>");
    gtk_box_pack_start(GTK_BOX(box), hint, FALSE, FALSE, 0);

    /* 初始值：手动档沿用当前设定，自动档以 EC 实际占空比为起点 */
    int init = (share_info->auto_duty == 0
            && share_info->manual_next_fan_duty >= FAN_MANUAL_MIN)
            ? share_info->manual_next_fan_duty
            : share_info->fan_duty;
    if (init < FAN_MANUAL_MIN)
        init = FAN_MANUAL_MIN;
    if (init > 100)
        init = 100;
    custom_pending_duty = init;
    gtk_range_set_value(GTK_RANGE(scale), init);
    ui_custom_set_value_label(init);

    g_signal_connect(scale, "value-changed", G_CALLBACK(ui_custom_changed), NULL);
    g_signal_connect(win, "destroy", G_CALLBACK(ui_custom_destroy), NULL);

    custom_window = win;
    gtk_widget_show_all(win);
    gtk_window_present(GTK_WINDOW(win));
}

/* ---------- 温度 / 转速悬浮窗 ----------
 * SNI 菜单项只能显示文字，放不下实时读数，所以另开一块无边框卡片：
 * 置顶、半透明、可拖动，位置和开关状态持久化到 ~/.config。*/

static void ui_load_css(void) {
    GtkCssProvider* provider = gtk_css_provider_new();
    /* 用自绘背景（见 ui_overlay_draw），这里把主题背景清掉，
     * 否则无边框窗口会带一圈默认的灰色描边。*/
    gtk_css_provider_load_from_data(provider,
            "window.clevo-overlay, window.clevo-overlay.background {"
            "  background-color: transparent;"
            "  background-image: none;"
            "  box-shadow: none;"
            "  border: none;"
            "}\n"
            /* 等宽数字：温度/转速位数变化时卡片宽度不跳 */
            "window.clevo-overlay label { font-feature-settings: 'tnum'; }\n",
            -1, NULL);
    gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
            GTK_STYLE_PROVIDER(provider),
            GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(provider);
}

static char* ui_overlay_config_path(void) {
    static char path[512];
    const char* xdg = getenv("XDG_CONFIG_HOME");
    const char* home = getenv("HOME");
    if (xdg != NULL && *xdg != '\0')
        snprintf(path, sizeof(path), "%s/clevo-indicator/overlay", xdg);
    else if (home != NULL && *home != '\0')
        snprintf(path, sizeof(path), "%s/.config/clevo-indicator/overlay", home);
    else
        return NULL;
    return path;
}

static void ui_overlay_config_save(int visible, int x, int y) {
    char* p = ui_overlay_config_path();
    if (p == NULL)
        return;
    char dir[512];
    snprintf(dir, sizeof(dir), "%s", p);
    char* slash = strrchr(dir, '/');
    if (slash != NULL) {
        *slash = '\0';
        mkdir(dir, 0755);
    }
    FILE* fp = fopen(p, "w");
    if (fp == NULL)
        return;
    fprintf(fp, "%d\n%d %d\n", visible, x, y);
    fclose(fp);
}

static int ui_overlay_config_load(int* visible, int* x, int* y) {
    if (visible != NULL)
        *visible = 0;
    if (x != NULL)
        *x = -1;
    if (y != NULL)
        *y = -1;
    char* p = ui_overlay_config_path();
    if (p == NULL)
        return 0;
    FILE* fp = fopen(p, "r");
    if (fp == NULL)
        return 0;
    int vis = 0, px = -1, py = -1;
    int n = fscanf(fp, "%d %d %d", &vis, &px, &py);
    fclose(fp);
    if (n < 1)
        return 0;
    if (visible != NULL)
        *visible = vis ? 1 : 0;
    if (n >= 3) {
        if (x != NULL)
            *x = px;
        if (y != NULL)
            *y = py;
    }
    return 1;
}

/* 温度越高越醒目：正常灰白、偏热琥珀、过热红 */
static const char* ui_overlay_temp_color(int t) {
    if (t >= 88)
        return "#ff5f56";
    if (t >= 75)
        return "#ffb454";
    return "#eaeaea";
}

/* 一格 = 小号说明 + 大号数值（+ 可选小号尾注）。
 * tail_out 非空时额外生成一个尾注标签，供动态改文字。*/
static GtkWidget* ui_overlay_cell(const char* caption, GtkWidget** value_out,
        int value_width, const char* tail, GtkWidget** tail_out) {
    GtkWidget* box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    char buf[128];
    snprintf(buf, sizeof(buf), "<span size='small' color='#8b9199'>%s</span>",
            caption);
    GtkWidget* cap = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(cap), buf);
    /* GtkLabel 内部默认居中（xalign=0.5），会让数值在定宽框里飘到中间，
     * 和前面的说明拉开距离，所以显式左对齐 */
    gtk_label_set_xalign(GTK_LABEL(cap), 0.0f);
    gtk_widget_set_halign(cap, GTK_ALIGN_START);
    gtk_widget_set_valign(cap, GTK_ALIGN_END);
    gtk_box_pack_start(GTK_BOX(box), cap, FALSE, FALSE, 0);

    GtkWidget* val = gtk_label_new(NULL);
    gtk_label_set_xalign(GTK_LABEL(val), 0.0f);
    gtk_widget_set_halign(val, GTK_ALIGN_START);
    gtk_widget_set_valign(val, GTK_ALIGN_END);
    gtk_widget_set_size_request(val, value_width, -1);
    gtk_box_pack_start(GTK_BOX(box), val, FALSE, FALSE, 0);
    *value_out = val;

    if (tail != NULL) {
        GtkWidget* t = gtk_label_new(NULL);
        gtk_label_set_xalign(GTK_LABEL(t), 0.0f);
        gtk_widget_set_halign(t, GTK_ALIGN_START);
        gtk_widget_set_valign(t, GTK_ALIGN_END);
        gtk_box_pack_start(GTK_BOX(box), t, FALSE, FALSE, 0);
        if (tail_out != NULL)
            *tail_out = t;
    }
    return box;
}

static gboolean ui_overlay_draw(GtkWidget* widget, cairo_t* cr,
        gpointer user_data) {
    GtkAllocation alloc;
    gtk_widget_get_allocation(widget, &alloc);
    double w = alloc.width, h = alloc.height, r = 13.0;
    if (w <= 2 * r || h <= 2 * r)
        return FALSE;
    cairo_new_sub_path(cr);
    cairo_arc(cr, w - r, r, r, -M_PI / 2.0, 0);
    cairo_arc(cr, w - r, h - r, r, 0, M_PI / 2.0);
    cairo_arc(cr, r, h - r, r, M_PI / 2.0, M_PI);
    cairo_arc(cr, r, r, r, M_PI, 1.5 * M_PI);
    cairo_close_path(cr);
    cairo_set_source_rgba(cr, 0.06, 0.07, 0.09, 0.86);
    cairo_fill_preserve(cr);
    cairo_set_line_width(cr, 1.0);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.14);
    cairo_stroke(cr);
    return FALSE;   /* 继续绘制子控件（文字） */
}

static gboolean ui_overlay_button_press(GtkWidget* widget, GdkEventButton* event,
        gpointer user_data) {
    if (event->button == 1) {
        /* 双击直接打开滑块窗口，省得再去翻托盘菜单 */
        if (event->type == GDK_2BUTTON_PRESS) {
            overlay_dragging = FALSE;
            ui_command_custom(0L);
            return TRUE;
        }
        int wx = 0, wy = 0;
        gtk_window_get_position(GTK_WINDOW(widget), &wx, &wy);
        overlay_dragging = TRUE;
        overlay_drag_dx = (int) event->x_root - wx;
        overlay_drag_dy = (int) event->y_root - wy;
        return TRUE;
    }
    if (event->button == 3 && indicator_menu != NULL) {
        ui_refresh_menuitems();
#if GTK_CHECK_VERSION(3, 22, 0)
        gtk_menu_popup_at_pointer(GTK_MENU(indicator_menu), (GdkEvent*) event);
#else
        gtk_menu_popup(GTK_MENU(indicator_menu), NULL, NULL, NULL, NULL,
                event->button, event->time);
#endif
        return TRUE;
    }
    return FALSE;
}

static gboolean ui_overlay_motion(GtkWidget* widget, GdkEventMotion* event,
        gpointer user_data) {
    if (!overlay_dragging)
        return FALSE;
    gtk_window_move(GTK_WINDOW(widget), (int) event->x_root - overlay_drag_dx,
            (int) event->y_root - overlay_drag_dy);
    return TRUE;
}

static gboolean ui_overlay_button_release(GtkWidget* widget,
        GdkEventButton* event, gpointer user_data) {
    if (event->button == 1 && overlay_dragging) {
        overlay_dragging = FALSE;
        int wx = 0, wy = 0;
        gtk_window_get_position(GTK_WINDOW(widget), &wx, &wy);
        overlay_last_x = wx;
        overlay_last_y = wy;
        ui_overlay_config_save(1, wx, wy);
        return TRUE;
    }
    return FALSE;
}

static void ui_overlay_destroy(GtkWidget* widget, gpointer user_data) {
    overlay_window = NULL;
    overlay_cpu_val = NULL;
    overlay_gpu_val = NULL;
    overlay_fan_mode = NULL;
    overlay_duty_val = NULL;
    overlay_rpm_val = NULL;
    overlay_dragging = FALSE;
}

static void ui_overlay_show(void) {
    if (overlay_window != NULL) {
        gtk_window_present(GTK_WINDOW(overlay_window));
        return;
    }

    int vis = 0, px = -1, py = -1;
    ui_overlay_config_load(&vis, &px, &py);

    GtkWidget* win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(win), "clevo-overlay");
    gtk_window_set_decorated(GTK_WINDOW(win), FALSE);
    gtk_window_set_resizable(GTK_WINDOW(win), FALSE);
    gtk_window_set_keep_above(GTK_WINDOW(win), TRUE);
    gtk_window_set_skip_taskbar_hint(GTK_WINDOW(win), TRUE);
    gtk_window_set_skip_pager_hint(GTK_WINDOW(win), TRUE);
    /* 不抢焦点：点它不会把正在用的窗口踢到后面 */
    gtk_window_set_accept_focus(GTK_WINDOW(win), FALSE);
    gtk_window_set_focus_on_map(GTK_WINDOW(win), FALSE);
    gtk_window_set_type_hint(GTK_WINDOW(win), GDK_WINDOW_TYPE_HINT_UTILITY);
    /* 圆角 + 半透明需要 RGBA visual，否则圆角外会是一块黑色方块 */
    GdkScreen* screen = gtk_widget_get_screen(win);
    GdkVisual* visual = gdk_screen_get_rgba_visual(screen);
    if (visual != NULL && gdk_screen_is_composited(screen))
        gtk_widget_set_visual(win, visual);
    gtk_widget_set_app_paintable(win, TRUE);
    gtk_style_context_add_class(gtk_widget_get_style_context(win),
            "clevo-overlay");

    GtkWidget* grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(grid), 16);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 2);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 14);
    gtk_container_add(GTK_CONTAINER(win), grid);

    /* 每个格子都是「小号说明 + 大号数值」，说明和数值在同一个盒子里。
     * 不用「说明一列、数值一列」的网格：那样列宽会被最长的那一格撑开，
     * GPU 的 -- 会飘到右边空出一大块。*/
    GtkWidget* cell;

    cell = ui_overlay_cell("CPU", &overlay_cpu_val, 78, NULL, NULL);
    gtk_grid_attach(GTK_GRID(grid), cell, 0, 0, 1, 1);

    cell = ui_overlay_cell("GPU", &overlay_gpu_val, 78, NULL, NULL);
    gtk_grid_attach(GTK_GRID(grid), cell, 1, 0, 1, 1);

    cell = ui_overlay_cell("风扇", &overlay_duty_val, 78, "自动",
            &overlay_fan_mode);
    gtk_grid_attach(GTK_GRID(grid), cell, 0, 1, 1, 1);

    cell = ui_overlay_cell("转速", &overlay_rpm_val, 78, NULL, NULL);
    gtk_grid_attach(GTK_GRID(grid), cell, 1, 1, 1, 1);

    gtk_widget_add_events(win, GDK_BUTTON_PRESS_MASK
            | GDK_BUTTON_RELEASE_MASK | GDK_BUTTON1_MOTION_MASK);

    g_signal_connect(win, "draw", G_CALLBACK(ui_overlay_draw), NULL);
    g_signal_connect(win, "button-press-event",
            G_CALLBACK(ui_overlay_button_press), NULL);
    g_signal_connect(win, "button-release-event",
            G_CALLBACK(ui_overlay_button_release), NULL);
    g_signal_connect(win, "motion-notify-event", G_CALLBACK(ui_overlay_motion),
            NULL);
    g_signal_connect(win, "destroy", G_CALLBACK(ui_overlay_destroy), NULL);

    overlay_window = win;
    gtk_widget_show_all(win);
    ui_overlay_update();

    /* 屏幕区域：既用于默认摆放，也用于把跑出屏幕的旧坐标拉回来
     * （卡片宽度会随数值位数变化，旧位置可能已经贴边）*/
    int sx = 0, sy = 0, sw = 1920, sh = 1080;
#if GTK_CHECK_VERSION(3, 22, 0)
    GdkDisplay* disp = gdk_display_get_default();
    GdkMonitor* mon = gdk_display_get_primary_monitor(disp);
    if (mon == NULL)
        mon = gdk_display_get_monitor(disp, 0);
    if (mon != NULL) {
        GdkRectangle geo;
        gdk_monitor_get_geometry(mon, &geo);
        sx = geo.x;
        sy = geo.y;
        sw = geo.width;
        sh = geo.height;
    }
#else
    sw = gdk_screen_get_width(screen);
    sh = gdk_screen_get_height(screen);
#endif

    int w = 0, h = 0;
    gtk_window_get_size(GTK_WINDOW(win), &w, &h);

    if (px >= 0 && py >= 0) {
        overlay_last_x = px;
        overlay_last_y = py;
    } else {
        /* 默认丢到主屏右上角，避开托盘和顶部面板 */
        overlay_last_x = sx + sw - w - 40;
        overlay_last_y = sy + 56;
    }
    if (overlay_last_x + w > sx + sw)
        overlay_last_x = sx + sw - w - 20;
    if (overlay_last_y + h > sy + sh)
        overlay_last_y = sy + sh - h - 20;
    if (overlay_last_x < sx)
        overlay_last_x = sx + 20;
    if (overlay_last_y < sy)
        overlay_last_y = sy + 20;
    gtk_window_move(GTK_WINDOW(win), overlay_last_x, overlay_last_y);
}

static void ui_overlay_update(void) {
    if (overlay_window == NULL)
        return;
    char buf[256];

    int t = (int) share_info->cpu_temp;
    snprintf(buf, sizeof(buf),
            "<span size='x-large' weight='bold' color='%s'>%d°C</span>",
            ui_overlay_temp_color(t), t);
    gtk_label_set_markup(GTK_LABEL(overlay_cpu_val), buf);

    int gt = (int) share_info->gpu_temp;
    if (gt > 0) {
        snprintf(buf, sizeof(buf),
                "<span size='x-large' weight='bold' color='%s'>%d°C</span>",
                ui_overlay_temp_color(gt), gt);
    } else {
        /* 本机 EC 不上报 GPU 二极管，显示 -- 而不是假温度 */
        snprintf(buf, sizeof(buf),
                "<span size='x-large' weight='bold' color='#5c626b'>--</span>");
    }
    gtk_label_set_markup(GTK_LABEL(overlay_gpu_val), buf);

    if (share_info->auto_duty)
        gtk_label_set_markup(GTK_LABEL(overlay_fan_mode),
                "<span size='small' color='#8b9199'>自动</span>");
    else
        gtk_label_set_markup(GTK_LABEL(overlay_fan_mode),
                "<span size='small' color='#d9a066'>手动</span>");

    /* 当前档位：EC 回读的实际 PWM 占空比 */
    snprintf(buf, sizeof(buf),
            "<span size='x-large' weight='bold' color='#8ab4f8'>%d%%</span>",
            (int) share_info->fan_duty);
    gtk_label_set_markup(GTK_LABEL(overlay_duty_val), buf);

    if (share_info->fan2_rpms > 0) {
        snprintf(buf, sizeof(buf),
                "<span color='#c8ccd0'>%d/%d</span>"
                "<span size='small' color='#8b9199'> RPM</span>",
                (int) share_info->fan_rpms, (int) share_info->fan2_rpms);
    } else {
        snprintf(buf, sizeof(buf),
                "<span color='#c8ccd0'>%d</span>"
                "<span size='small' color='#8b9199'> RPM</span>",
                (int) share_info->fan_rpms);
    }
    gtk_label_set_markup(GTK_LABEL(overlay_rpm_val), buf);
}

static void ui_command_overlay(long ignored) {
    if (overlay_window != NULL) {
        ui_overlay_config_save(0, overlay_last_x, overlay_last_y);
        gtk_widget_destroy(overlay_window);   /* destroy 回调会清空指针 */
    } else {
        ui_overlay_show();
        ui_overlay_config_save(1, overlay_last_x, overlay_last_y);
    }
    ui_refresh_menuitems();
}

static void ui_command_set_epp(long which) {
    int v = (int) which;
    printf("clicked on cpu power mode: %d\n", v);
    share_info->epp_request = v;
    share_info->epp_state = v;
    epp_config_save(v);
    ui_refresh_menuitems();
}

static void ui_on_menu_show(GtkWidget* widget, gpointer data) {
    ui_refresh_menuitems();
}

static void ui_refresh_menuitems(void) {
    for (int i = 0; i < menuitem_count; i++) {
        if (menuitems[i].widget == NULL || menuitems[i].type == NA)
            continue;
        int active = 0;
        switch (menuitems[i].type) {
        case EPP:
            active = (share_info->epp_state == (int) menuitems[i].option);
            break;
        case AUTO:
            active = (share_info->auto_duty == 1);
            break;
        case MANUAL:
            active = (share_info->auto_duty == 0
                    && share_info->manual_next_fan_duty
                            == (int) menuitems[i].option);
            break;
        case CUSTOM: {
            int d = share_info->manual_next_fan_duty;
            active = (share_info->auto_duty == 0 && d > 0);
            /* 占空比正好落在某个预设档上时，勾选归对应的预设项 */
            for (int j = 0; j < menuitem_count && active; j++) {
                if (menuitems[j].type == MANUAL
                        && (int) menuitems[j].option == d)
                    active = 0;
            }
            break;
        }
        case OVERLAY:
            active = (overlay_window != NULL);
            break;
        default:
            break;
        }
        char itemlabel[256];
        if (menuitems[i].type == CUSTOM && active) {
            snprintf(itemlabel, sizeof(itemlabel), "风扇：自定义…（当前 %d%%）",
                    (int) share_info->manual_next_fan_duty);
        } else if (menuitems[i].type == OVERLAY) {
            snprintf(itemlabel, sizeof(itemlabel), "悬浮窗：%s",
                    active ? "隐藏温度与转速" : "显示温度与转速");
        } else {
            snprintf(itemlabel, sizeof(itemlabel), "%s", menuitems[i].label);
        }
        char buf[300];
        snprintf(buf, sizeof(buf), "%s %s", active ? "●" : "○", itemlabel);
        gtk_menu_item_set_label(GTK_MENU_ITEM(menuitems[i].widget), buf);
    }
}

/* ---------- CPU 电源偏好（intel_pstate HWP EPP） ----------
 * 写 sysfs 需要 root，因此由 worker 进程执行，UI 只通过共享内存发请求。*/

static int sysfs_apply_epp(int which) {
    const char* val = (which == EPP_POWER) ? "power" : "balance_performance";
    char path[256];
    int applied = 0;
    for (int i = 0; i < 64; i++) {
        snprintf(path, sizeof(path),
                "/sys/devices/system/cpu/cpufreq/policy%d/energy_performance_preference",
                i);
        int fd = open(path, O_WRONLY);
        if (fd < 0)
            continue;   /* 该编号不存在或不可写 */
        ssize_t n = write(fd, val, strlen(val));
        close(fd);
        if (n == (ssize_t) strlen(val))
            applied++;
    }
    return applied;
}

static int sysfs_query_epp(void) {
    char buf[64] = {0};
    int fd = open(
            "/sys/devices/system/cpu/cpufreq/policy0/energy_performance_preference",
            O_RDONLY);
    if (fd < 0)
        return EPP_DEFAULT;
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return EPP_DEFAULT;
    return (strncmp(buf, "power", 5) == 0) ? EPP_POWER : EPP_DEFAULT;
}

static char* epp_config_path(void) {
    static char path[512];
    const char* xdg = getenv("XDG_CONFIG_HOME");
    const char* home = getenv("HOME");
    if (xdg != NULL && *xdg != '\0')
        snprintf(path, sizeof(path), "%s/clevo-indicator/epp", xdg);
    else if (home != NULL && *home != '\0')
        snprintf(path, sizeof(path), "%s/.config/clevo-indicator/epp", home);
    else
        return NULL;
    return path;
}

static void epp_config_save(int which) {
    char* p = epp_config_path();
    if (p == NULL)
        return;
    char dir[512];
    snprintf(dir, sizeof(dir), "%s", p);
    char* slash = strrchr(dir, '/');
    if (slash != NULL) {
        *slash = '\0';
        mkdir(dir, 0755);
    }
    FILE* fp = fopen(p, "w");
    if (fp == NULL)
        return;
    fprintf(fp, "%s\n", which == EPP_POWER ? "power" : "default");
    fclose(fp);
}

static int epp_config_load(void) {
    char* p = epp_config_path();
    if (p == NULL)
        return -1;
    FILE* fp = fopen(p, "r");
    if (fp == NULL)
        return -1;
    char buf[32] = {0};
    int rc = -1;
    if (fgets(buf, sizeof(buf), fp) != NULL) {
        if (strncmp(buf, "power", 5) == 0)
            rc = EPP_POWER;
        else if (strncmp(buf, "default", 7) == 0)
            rc = EPP_DEFAULT;
    }
    fclose(fp);
    return rc;
}

static int ec_init(void) {
    if (ioperm(EC_DATA, 1, 1) != 0)
        return EXIT_FAILURE;
    if (ioperm(EC_SC, 1, 1) != 0)
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}

static void ec_on_sigterm(int signum) {
    printf("ec on signal: %s\n", strsignal(signum));
    if (share_info != NULL)
        share_info->exit = 1;
}

/* 非对称滞后调速：
 *   升速 —— 用瞬时温度 temp_up，快速响应，保护硬件
 *   降速 —— 用长期平均温度 temp_down（约 6 秒窗口），延迟响应，防止抖动
 */
/* 档位切换死区：进出同一档位需要多走 3°C，
 * 否则温度停在档位边界（如 60°C）时会每 200ms 反复跳档。*/
#define AUTO_HYST 3

/* 温度 → 占空比 映射表。
 * 原曲线的阈值是按「空闲 30°C 左右」的机器写的，本机空闲就有 45–52°C，
 * 于是自动模式长期停在 70–80%，永远降不下来。现按实测重标：
 * 空闲 46–50°C→28–36%（约 2200–2900 RPM），84°C 以上才逐步拉到 90–100%。*/
static int ec_auto_duty_adjust(int temp_up, int temp_down) {
    static const int duty_steps[] = { 18, 22, 28, 36, 48, 62, 78, 90, 100 };
    static const int thresholds[] = { 36, 44, 52, 60, 68, 76, 84, 90 };
    int last = (int) (sizeof(duty_steps) / sizeof(duty_steps[0])) - 1;

    /* 用「上次写入值」而非 EC 回读值作比较基准：
     * 回读是 0–255 量化后的百分比（写 28% 回读 27%），
     * 用它比较会导致每 200ms 重复写一次 EC。*/
    int cur = share_info->auto_duty_val;
    if (cur == 0)
        cur = share_info->fan_duty;     /* 刚切到自动模式，以 EC 实际值为起点 */

    if (temp_up >= 92)
        return 100;                     /* 急热：无视当前档位直接全速 */

    /* 定位当前档位（不高于当前占空比的最大档） */
    int idx = 0;
    for (int i = 0; i <= last; i++) {
        if (cur >= duty_steps[i])
            idx = i;
    }

    /* 升温：瞬时温度越过「上一档门槛 + 死区」才升，可能连跳多档 */
    int up = idx;
    while (up < last && temp_up >= thresholds[up] + AUTO_HYST)
        up++;
    if (up > idx)
        return duty_steps[up];

    /* 降温：平滑温度低于「本档门槛 - 死区」才降 */
    int dn = idx;
    while (dn > 0 && temp_down <= thresholds[dn - 1] - AUTO_HYST)
        dn--;
    if (dn < idx)
        return duty_steps[dn];

    return 0;
}

static int ec_query_cpu_temp(void) {
    return ec_io_read(EC_REG_CPU_TEMP);
}

static int ec_query_gpu_temp(void) {
    return ec_io_read(EC_REG_GPU_TEMP);
}

static int ec_query_fan_duty(void) {
    int raw_duty = ec_io_read(EC_REG_FAN_DUTY);
    return calculate_fan_duty(raw_duty);
}

static int ec_query_fan_rpms(void) {
    int raw_rpm_hi = ec_io_read(EC_REG_FAN_RPMS_HI);
    int raw_rpm_lo = ec_io_read(EC_REG_FAN_RPMS_LO);
    return calculate_fan_rpms(raw_rpm_hi, raw_rpm_lo);
}

static int ec_query_fan2_rpms(void) {
    int raw_rpm_hi = ec_io_read(EC_REG_FAN2_RPMS_HI);
    int raw_rpm_lo = ec_io_read(EC_REG_FAN2_RPMS_LO);
    return calculate_fan_rpms(raw_rpm_hi, raw_rpm_lo);
}

/*
 * Write fan duty to both CPU fan (port 0x01) and GPU fan (port 0x02).
 * Both fans are always kept in sync.
 */
static int ec_write_fan_duty(int duty_percentage) {
    if (duty_percentage < FAN_MANUAL_MIN || duty_percentage > 100) {
        printf("Wrong fan duty to write: %d\n", duty_percentage);
        return EXIT_FAILURE;
    }
    double v_d = ((double) duty_percentage) / 100.0 * 255.0;
    int v_i = (int) v_d;
    int ret = ec_io_do(EC_CMD_FAN, EC_PORT_FAN_CPU, v_i);
    if (ret != EXIT_SUCCESS) return ret;
    return ec_io_do(EC_CMD_FAN, EC_PORT_FAN_GPU, v_i);
}

static int ec_io_wait(const uint32_t port, const uint32_t flag,
        const char value) {
    uint8_t data = inb(port);
    int i = 0;
    while ((((data >> flag) & 0x1) != value) && (i++ < 100)) {
        usleep(1000);
        data = inb(port);
    }
    if (i >= 100) {
        printf("wait_ec error on port 0x%x, data=0x%x, flag=0x%x, value=0x%x\n",
                port, data, flag, value);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static uint8_t ec_io_read(const uint32_t port) {
    ec_io_wait(EC_SC, IBF, 0);
    outb(EC_SC_READ_CMD, EC_SC);

    ec_io_wait(EC_SC, IBF, 0);
    outb(port, EC_DATA);

    ec_io_wait(EC_SC, OBF, 1);
    uint8_t value = inb(EC_DATA);

    return value;
}

static int ec_io_do(const uint32_t cmd, const uint32_t port,
        const uint8_t value) {
    ec_io_wait(EC_SC, IBF, 0);
    outb(cmd, EC_SC);

    ec_io_wait(EC_SC, IBF, 0);
    outb(port, EC_DATA);

    ec_io_wait(EC_SC, IBF, 0);
    outb(value, EC_DATA);

    return ec_io_wait(EC_SC, IBF, 0);
}

static int calculate_fan_duty(int raw_duty) {
    return (int) ((double) raw_duty / 255.0 * 100.0);
}

static int calculate_fan_rpms(int raw_rpm_high, int raw_rpm_low) {
    int raw_rpm = (raw_rpm_high << 8) + raw_rpm_low;
    return raw_rpm > 0 ? (2156220 / raw_rpm) : 0;
}

static int check_proc_instances(const char* proc_name) {
    int proc_name_len = strlen(proc_name);
    pid_t this_pid = getpid();
    DIR* dir;
    if (!(dir = opendir("/proc"))) {
        perror("can't open /proc");
        return -1;
    }
    int instance_count = 0;
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        char* endptr;
        long lpid = strtol(ent->d_name, &endptr, 10);
        if (*endptr != '\0')
            continue;
        if (lpid == this_pid)
            continue;
        char buf[512];
        snprintf(buf, sizeof(buf), "/proc/%ld/comm", lpid);
        FILE* fp = fopen(buf, "r");
        if (fp) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                if ((buf[proc_name_len] == '\n' || buf[proc_name_len] == '\0')
                        && strncmp(buf, proc_name, proc_name_len) == 0) {
                    fprintf(stderr, "Process: %ld\n", lpid);
                    instance_count += 1;
                }
            }
            fclose(fp);
        }
    }
    closedir(dir);
    return instance_count;
}

static void get_time_string(char* buffer, size_t max, const char* format) {
    time_t timer;
    struct tm tm_info;
    time(&timer);
    localtime_r(&timer, &tm_info);
    strftime(buffer, max, format, &tm_info);
}

static void signal_term(__sighandler_t handler) {
    signal(SIGHUP, handler);
    signal(SIGINT, handler);
    signal(SIGQUIT, handler);
    signal(SIGPIPE, handler);
    signal(SIGALRM, handler);
    signal(SIGTERM, handler);
    signal(SIGUSR1, handler);
    signal(SIGUSR2, handler);
}
