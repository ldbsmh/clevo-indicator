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
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdarg.h>
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
static GtkWidget* ui_overlay_cap(const char* text);
static GtkWidget* ui_overlay_row(GtkOrientation orientation);
static GtkWidget* ui_overlay_ring(void);
static void ui_overlay_ring_set(GtkWidget* ring, double frac,
        const char* color);
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
static int curve_auto_target(int cpu_temp, int gpu_temp);
static int curve_ctrl_temperature(void);
static const char* curve_reason(void);
static void curve_load(void);
static int main_print_curve(void);
static int ec_query_cpu_temp(void);
static int ec_query_gpu_temp(void);
static int nvml_gpu_temperature(void);
static int gpu_temperature_any(void);
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
static void ec_log(const char* format, ...);
static void ec_freq_context(char* buf, size_t size);
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
static GtkWidget* overlay_cpu_val = NULL;      /* 顶部大字：CPU 温度 */
static GtkWidget* overlay_cpu_tail = NULL;     /* 顶部右侧：档位 + 自动/手动 */
static GtkWidget* overlay_gpu_val = NULL;
static GtkWidget* overlay_gpu_tail = NULL;     /* GPU 风扇转速 */
static GtkWidget* overlay_fan_mode = NULL;     /* 档位行的 自动/手动 */
static GtkWidget* overlay_duty_val = NULL;
static GtkWidget* overlay_rpm_val = NULL;
static GtkWidget* overlay_ring_gpu = NULL;
static GtkWidget* overlay_ring_duty = NULL;
static GtkWidget* overlay_ring_rpm = NULL;
static GtkWidget* overlay_spark = NULL;

/* 折线用的温度历史：每秒采一个点，保留 60 秒 */
#define OVERLAY_SPARK_POINTS 60
static int overlay_spark_hist[OVERLAY_SPARK_POINTS];
static int overlay_spark_len = 0;
static int overlay_spark_pos = 0;
static int overlay_spark_tick = 0;

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
    curve_load();
    /* 只读曲线、不需要碰 EC，放在单实例检查之前：
     * 服务已经在跑的时候也能随时查看曲线长什么样。*/
    if (argc > 1 && strcmp(argv[1], "--curve") == 0)
        return main_print_curve();
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
  --curve\t\t\tPrint the active fan curve table and exit\n\
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
\n\
The automatic fan curve can be overridden in\n\
~/.config/clevo-indicator/curve, for example:\n\
\n\
  cpu 40:18 48:22 56:28 64:40 72:54 78:68 84:84 89:100\n\
  gpu 40:18 55:22 62:30 70:45 76:60 80:75 85:90 88:100\n\
  hyst 3            # 降温需要比入口温度低 3°C 才降档\n\
  critical 92       # 超过 92°C 无条件全速\n\
  critical_offset 6 # 回落到 86°C 才交还曲线\n\
  ramp_up 20        # 单步最多升 20%%\n\
  ramp_down 5       # 单步最多降 5%%\n\
  mode linear       # linear | step\n\
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
    if (share_info->epp_state < 0)
        share_info->epp_state = EPP_DEFAULT;
    share_info->epp_request = -1;
}

static int main_ec_worker(void) {
    setuid(0);
    system("modprobe ec_sys");
    int epp_ticks = 0;
    int manual_thermal_override = 0;
    int gpu_ticks = 4;   /* 首轮循环就立刻取一次 */
    int gpu_cached = 0;
    while (share_info->exit == 0) {
        // check parent
        if (parent_pid != 0 && kill(parent_pid, 0) == -1) {
            ec_log("worker on parent death\n");
            break;
        }
        // CPU 电源偏好：UI 只负责发请求，实际写 sysfs 必须由 root 完成
        int epp_req = share_info->epp_request;
        if (epp_req >= 0) {
            int applied = sysfs_apply_epp(epp_req);
            if (applied > 0) {
                ec_log("CPU 电源偏好 -> %s（%d 个核心）\n",
                        epp_req == EPP_POWER ? "power" : "balance_performance",
                        applied);
                share_info->epp_state = epp_req;
            } else {
                ec_log("写入 CPU 电源偏好失败（需要 root 权限）\n");
            }
            share_info->epp_request = -1;
        } else if (++epp_ticks >= 10) {
            /* 每 2 秒核对一次。息屏、唤醒、固件自检等事件会把这个值打回
             * balance_performance；原来只是回读、跟着显示，于是菜单就
             * 「变回默认项」了。现在改成守住用户的选择：发现被改掉就立刻
             * 写回，并把时间和前后值记进日志，方便追是谁改的。*/
            epp_ticks = 0;
            int want = share_info->epp_state;
            int actual = sysfs_query_epp();
            if (actual >= 0 && actual != want) {
                int applied = sysfs_apply_epp(want);
                if (applied > 0) {
                    char s_time[256];
                    char ctx[128];
                    get_time_string(s_time, 256, "%m/%d %H:%M:%S");
                    ec_freq_context(ctx, sizeof(ctx));
                    ec_log("%s CPU 电源偏好被外部改为 %s，已纠回 %s"
                            "（%d 核，%s）\n", s_time,
                            actual == EPP_POWER ? "power" : "balance_performance",
                            want == EPP_POWER ? "power" : "balance_performance",
                            applied, ctx);
                } else {
                    /* 写不进去（权限/接口变化）就跟随实际值，避免菜单说谎 */
                    share_info->epp_state = actual;
                }
            }
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
            ec_log("unable to read EC from sysfs: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        unsigned char buf[EC_REG_SIZE];
        ssize_t len = read(io_fd, buf, EC_REG_SIZE);
        switch (len) {
        case -1:
            ec_log("unable to read EC from sysfs: %s\n", strerror(errno));
            break;
        case 0x100:
            share_info->cpu_temp = buf[EC_REG_CPU_TEMP];
            /* 部分机型（如混合显卡模式的 T1000）EC 侧 GPU 二极管
             * 不上报，0xCD 恒为 0。此时改用 NVML 向 NVIDIA 驱动要温度，
             * 每秒取一次，其余循环沿用缓存值；两者都拿不到才显示 --。*/
            {
                int gpu = buf[EC_REG_GPU_TEMP];
                if (gpu > 0 && gpu < 110) {
                    share_info->gpu_temp = gpu;
                } else {
                    if (++gpu_ticks >= 5) {
                        gpu_ticks = 0;
                        int t = nvml_gpu_temperature();
                        gpu_cached = (t > 0) ? t : 0;
                    }
                    share_info->gpu_temp = gpu_cached;
                }
            }
            share_info->fan_duty = calculate_fan_duty(buf[EC_REG_FAN_DUTY]);
            share_info->fan_rpms = calculate_fan_rpms(buf[EC_REG_FAN_RPMS_HI],
                    buf[EC_REG_FAN_RPMS_LO]);
            share_info->fan2_rpms = calculate_fan_rpms(buf[EC_REG_FAN2_RPMS_HI],
                    buf[EC_REG_FAN2_RPMS_LO]);
            break;
        default:
            ec_log("wrong EC size from sysfs: %ld\n", len);
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
                ec_log("%s 手动档 %d%% 但温度 %d°C 超限，临时全速\n", s_time,
                        share_info->manual_next_fan_duty, t);
                ec_write_fan_duty(100);
                manual_thermal_override = 1;
            } else if (manual_thermal_override && t <= 80) {
                int target = share_info->manual_next_fan_duty;
                if (target >= FAN_MANUAL_MIN) {
                    char s_time[256];
                    get_time_string(s_time, 256, "%m/%d %H:%M:%S");
                    ec_log("%s 温度回落到 %d°C，恢复手动档 %d%%\n", s_time, t,
                            target);
                    ec_write_fan_duty(target);
                }
                manual_thermal_override = 0;
            }
        }
        // auto EC
        if (share_info->auto_duty == 1) {
            int next_duty = curve_auto_target(share_info->cpu_temp,
                    share_info->gpu_temp);
            if (next_duty != 0 && next_duty != share_info->auto_duty_val) {
                char s_time[256];
                get_time_string(s_time, 256, "%m/%d %H:%M:%S");
                ec_log("%s CPU=%d°C, GPU=%d°C, ctrl=%d°C, auto fan duty %d%% -> %d%%（%s）\n",
                        s_time, share_info->cpu_temp, share_info->gpu_temp,
                        curve_ctrl_temperature(),
                        share_info->auto_duty_val ? share_info->auto_duty_val
                                                  : share_info->fan_duty,
                        next_duty, curve_reason());
                ec_write_fan_duty(next_duty);
                share_info->auto_duty_val = next_duty;
            }
        }
        //
        usleep(200 * 1000);
    }
    ec_log("worker quit\n");
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
    int gpu_temp = gpu_temperature_any();
    printf("Dump fan information\n");
    printf("  CPU FAN Duty: %d%%\n", ec_query_fan_duty());
    printf("  CPU FAN RPMs: %d RPM\n", ec_query_fan_rpms());
    printf("  GPU FAN RPMs: %d RPM\n", ec_query_fan2_rpms());
    printf("  CPU Temp:     %d°C\n", ec_query_cpu_temp());
    if (gpu_temp > 0)
        printf("  GPU Temp:     %d°C\n", gpu_temp);
    else
        printf("  GPU Temp:     N/A (EC 与 NVML 均未提供)\n");
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
    /* 卡片外观全部走 CSS，只有外框圆角和描边用 cairo 自绘
     * （见 ui_overlay_draw），因为 GTK3 的 box-shadow 在透明窗口上不生效。*/
    gtk_css_provider_load_from_data(provider,
            "window.clevo-overlay, window.clevo-overlay.background {"
            "  background-color: transparent;"
            "  background-image: none;"
            "  box-shadow: none;"
            "  border: none;"
            "}\n"
            /* 等宽数字：温度/转速位数变化时卡片宽度不跳 */
            "window.clevo-overlay label { font-feature-settings: 'tnum'; }\n"
            /* 内层分块：比外框亮一点的白，形成玻璃叠加感 */
            "window.clevo-overlay .ov-row {"
            "  background-color: rgba(255, 255, 255, 0.058);"
            "  background-image: none;"
            "  border: none;"
            "  border-radius: 12px;"
            "  padding: 7px 11px;"
            "}\n"
            "window.clevo-overlay .ov-cap {"
            "  color: rgba(255, 255, 255, 0.52);"
            "  font-size: 9pt;"
            "  font-weight: 500;"
            "}\n"
            "window.clevo-overlay .ov-hero {"
            "  color: #f4f6f8;"
            "  font-size: 19pt;"
            "  font-weight: 500;"
            "}\n"
            "window.clevo-overlay .ov-val {"
            "  color: #f4f6f8;"
            "  font-size: 12pt;"
            "  font-weight: 500;"
            "}\n"
            "window.clevo-overlay .ov-sub {"
            "  color: rgba(255, 255, 255, 0.46);"
            "  font-size: 8pt;"
            "}\n",
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

/* 温度越高越醒目：正常近白、偏热琥珀、过热红 */
static const char* ui_overlay_temp_color(int t) {
    if (t >= 88)
        return "#ff6b60";
    if (t >= 75)
        return "#f2b63f";
    return "#eef1f4";
}

/* 圆环底色比数值柔和：正常用青绿，避免整块卡片太吵 */
static const char* ui_overlay_ring_temp_color(int t) {
    if (t >= 88)
        return "#ff6b60";
    if (t >= 75)
        return "#f2b63f";
    return "#4ec9a8";
}

/* ---------- 圆环指示器 ----------
 * 自绘的小组件：一圈淡轨 + 一段彩色弧，从 12 点方向顺时针。
 * 用 GtkDrawingArea 而不是字符画，是为了弧线端点能圆头、边缘抗锯齿。*/
typedef struct {
    double frac;
    double r, g, b;
} OverlayRing;

static gboolean ui_overlay_ring_draw(GtkWidget* widget, cairo_t* cr,
        gpointer user_data) {
    OverlayRing* ring = g_object_get_data(G_OBJECT(widget), "ov-ring");
    if (ring == NULL)
        return FALSE;
    GtkAllocation a;
    gtk_widget_get_allocation(widget, &a);
    double cx = a.width / 2.0, cy = a.height / 2.0;
    double radius = MIN(a.width, a.height) / 2.0 - 2.0;
    if (radius <= 1.0)
        return FALSE;

    cairo_set_line_width(cr, 3.0);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

    /* 轨道留 24° 缺口，彩色弧从缺口之后开始，视觉上有「进度」的起点 */
    const double gap = 14.0 * M_PI / 180.0;
    double start = -M_PI / 2.0 + gap / 2.0;
    double sweep = 2.0 * M_PI - gap;

    cairo_new_sub_path(cr);
    cairo_arc(cr, cx, cy, radius, start, start + sweep);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.17);
    cairo_stroke(cr);

    double f = ring->frac;
    if (f > 0.004) {
        if (f > 1.0)
            f = 1.0;
        cairo_new_sub_path(cr);
        cairo_arc(cr, cx, cy, radius, start, start + sweep * f);
        cairo_set_source_rgba(cr, ring->r, ring->g, ring->b, 0.95);
        cairo_stroke(cr);
    }
    return FALSE;
}

static void ui_overlay_ring_free(gpointer data) {
    g_free(data);
}

/* 自绘控件本身是独立 GdkWindow，会截住鼠标事件，
 * 导致在圆环/折线上拖动或右键都没反应。把输入形状清空，事件透给窗口。*/
static void ui_overlay_pass_through(GtkWidget* widget) {
    cairo_region_t* empty = cairo_region_create();
    gtk_widget_input_shape_combine_region(widget, empty);
    cairo_region_destroy(empty);
}

static GtkWidget* ui_overlay_ring(void) {
    GtkWidget* w = gtk_drawing_area_new();
    gtk_widget_set_size_request(w, 26, 26);
    gtk_widget_set_valign(w, GTK_ALIGN_CENTER);
    ui_overlay_pass_through(w);
    OverlayRing* ring = g_new0(OverlayRing, 1);
    ring->r = 0.31;
    ring->g = 0.79;
    ring->b = 0.66;
    g_object_set_data_full(G_OBJECT(w), "ov-ring", ring, ui_overlay_ring_free);
    g_signal_connect(w, "draw", G_CALLBACK(ui_overlay_ring_draw), NULL);
    return w;
}

static void ui_overlay_ring_set(GtkWidget* widget, double frac,
        const char* color) {
    if (widget == NULL)
        return;
    OverlayRing* ring = g_object_get_data(G_OBJECT(widget), "ov-ring");
    if (ring == NULL)
        return;
    ring->frac = frac;
    unsigned int r = 0, g = 0, b = 0;
    if (color != NULL && sscanf(color, "#%2x%2x%2x", &r, &g, &b) == 3) {
        ring->r = r / 255.0;
        ring->g = g / 255.0;
        ring->b = b / 255.0;
    }
    gtk_widget_queue_draw(widget);
}

/* ---------- 温度折线 ----------
 * 每秒采一个点、保留 60 秒，右侧最新。纵轴固定 34~100°C，
 * 不随数据自适应，否则温度平稳时噪声会被放大成锯齿。*/
static gboolean ui_overlay_spark_draw(GtkWidget* widget, cairo_t* cr,
        gpointer user_data) {
    GtkAllocation a;
    gtk_widget_get_allocation(widget, &a);
    int n = overlay_spark_len;
    if (n < 2 || a.width < 8 || a.height < 6)
        return FALSE;

    const double lo = 34.0, hi = 100.0;
    double x0 = 1.0, x1 = a.width - 1.0;
    double y0 = 2.0, y1 = a.height - 2.0;
    double xs[OVERLAY_SPARK_POINTS], ys[OVERLAY_SPARK_POINTS];

    for (int i = 0; i < n; i++) {
        int idx = (overlay_spark_pos - n + i + 2 * OVERLAY_SPARK_POINTS)
                % OVERLAY_SPARK_POINTS;
        double v = overlay_spark_hist[idx];
        if (v < lo)
            v = lo;
        if (v > hi)
            v = hi;
        xs[i] = x0 + (x1 - x0) * (double) i / (double) (n - 1);
        ys[i] = y1 - (y1 - y0) * (v - lo) / (hi - lo);
    }

    /* 线下渐变，让折线不至于像一条孤立的细丝 */
    cairo_move_to(cr, xs[0], y1);
    for (int i = 0; i < n; i++)
        cairo_line_to(cr, xs[i], ys[i]);
    cairo_line_to(cr, xs[n - 1], y1);
    cairo_close_path(cr);
    cairo_pattern_t* pat = cairo_pattern_create_linear(0, y0, 0, y1);
    cairo_pattern_add_color_stop_rgba(pat, 0.0, 1.0, 1.0, 1.0, 0.10);
    cairo_pattern_add_color_stop_rgba(pat, 1.0, 1.0, 1.0, 1.0, 0.0);
    cairo_set_source(cr, pat);
    cairo_fill(cr);
    cairo_pattern_destroy(pat);

    cairo_move_to(cr, xs[0], ys[0]);
    for (int i = 1; i < n; i++)
        cairo_line_to(cr, xs[i], ys[i]);
    cairo_set_line_width(cr, 1.3);
    cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.72);
    cairo_stroke(cr);

    /* 末端亮点，强调「当前值在这」 */
    cairo_arc(cr, xs[n - 1], ys[n - 1], 2.0, 0, 2.0 * M_PI);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 0.95);
    cairo_fill(cr);
    return FALSE;
}

static GtkWidget* ui_overlay_cap(const char* text) {
    GtkWidget* label = gtk_label_new(text);
    gtk_label_set_xalign(GTK_LABEL(label), 0.0f);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_widget_set_valign(label, GTK_ALIGN_CENTER);
    gtk_style_context_add_class(gtk_widget_get_style_context(label), "ov-cap");
    return label;
}

/* 一块内层卡片：半透明白底 + 圆角，叠在外框上形成层次 */
static GtkWidget* ui_overlay_row(GtkOrientation orientation) {
    GtkWidget* box = gtk_box_new(orientation, 8);
    gtk_style_context_add_class(gtk_widget_get_style_context(box), "ov-row");
    return box;
}

/* 数值标签：字号/颜色走 CSS，动态颜色时才用 markup 覆盖 */
static GtkWidget* ui_overlay_value(const char* cls) {
    GtkWidget* label = gtk_label_new(NULL);
    gtk_label_set_xalign(GTK_LABEL(label), 0.0f);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_widget_set_valign(label, GTK_ALIGN_CENTER);
    gtk_style_context_add_class(gtk_widget_get_style_context(label), cls);
    return label;
}

static gboolean ui_overlay_draw(GtkWidget* widget, cairo_t* cr,
        gpointer user_data) {
    GtkAllocation alloc;
    gtk_widget_get_allocation(widget, &alloc);
    double w = alloc.width, h = alloc.height, r = 18.0;
    if (w <= 2 * r || h <= 2 * r)
        return FALSE;
    cairo_new_sub_path(cr);
    cairo_arc(cr, w - r, r, r, -M_PI / 2.0, 0);
    cairo_arc(cr, w - r, h - r, r, 0, M_PI / 2.0);
    cairo_arc(cr, r, h - r, r, M_PI / 2.0, M_PI);
    cairo_arc(cr, r, r, r, M_PI, 1.5 * M_PI);
    cairo_close_path(cr);

    /* 上浅下深的竖向渐变，比纯色更像一块玻璃板 */
    cairo_pattern_t* bg = cairo_pattern_create_linear(0, 0, 0, h);
    cairo_pattern_add_color_stop_rgba(bg, 0.0, 0.115, 0.122, 0.14, 1.0);
    cairo_pattern_add_color_stop_rgba(bg, 1.0, 0.062, 0.067, 0.08, 1.0);
    cairo_set_source(cr, bg);
    cairo_fill_preserve(cr);
    cairo_pattern_destroy(bg);

    /* 顶部一条亮边模拟玻璃高光，底部淡一些 */
    cairo_pattern_t* edge = cairo_pattern_create_linear(0, 0, 0, h);
    cairo_pattern_add_color_stop_rgba(edge, 0.0, 1, 1, 1, 0.16);
    cairo_pattern_add_color_stop_rgba(edge, 1.0, 1, 1, 1, 0.05);
    cairo_set_line_width(cr, 1.0);
    cairo_set_source(cr, edge);
    cairo_stroke(cr);
    cairo_pattern_destroy(edge);
    return FALSE;   /* 继续绘制子控件 */
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
        /* 这里必须用 at_rect 锚到点击位置：at_pointer 在本窗口上
         * 不报错也不弹菜单（窗口不参与焦点管理），换成显式锚点后正常。*/
        GdkRectangle anchor = { (int) event->x, (int) event->y, 1, 1 };
        gtk_menu_popup_at_rect(GTK_MENU(indicator_menu),
                gtk_widget_get_window(widget), &anchor,
                GDK_GRAVITY_NORTH_WEST, GDK_GRAVITY_NORTH_WEST,
                (GdkEvent*) event);
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
    overlay_cpu_tail = NULL;
    overlay_gpu_val = NULL;
    overlay_gpu_tail = NULL;
    overlay_fan_mode = NULL;
    overlay_duty_val = NULL;
    overlay_rpm_val = NULL;
    overlay_ring_gpu = NULL;
    overlay_ring_duty = NULL;
    overlay_ring_rpm = NULL;
    overlay_spark = NULL;
    /* 关掉再打开时折线从零开始画，不要把上次的历史当成连续数据 */
    overlay_spark_len = 0;
    overlay_spark_pos = 0;
    overlay_spark_tick = 0;
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

    /* 卡片结构（自上而下）：
     *   ① CPU 温度大字 + 右侧档位，下面一条 60 秒温度走势
     *   ② GPU 温度，右侧 GPU 风扇转速
     *   ③ 风扇档位，右侧 自动/手动
     *   ④ 双风扇转速
     * 每块都是独立的半透明圆角小卡，叠在渐变外框上。*/
    GtkWidget* outer = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    gtk_container_set_border_width(GTK_CONTAINER(outer), 9);
    gtk_container_add(GTK_CONTAINER(win), outer);

    /* ① 主指标 */
    GtkWidget* hero = ui_overlay_row(GTK_ORIENTATION_VERTICAL);
    gtk_box_set_spacing(GTK_BOX(hero), 3);
    GtkWidget* hero_top = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
    gtk_box_pack_start(GTK_BOX(hero_top), ui_overlay_cap("CPU"), FALSE, FALSE,
            0);
    overlay_cpu_val = ui_overlay_value("ov-hero");
    gtk_box_pack_start(GTK_BOX(hero_top), overlay_cpu_val, FALSE, FALSE, 0);

    overlay_cpu_tail = gtk_label_new(NULL);
    gtk_label_set_xalign(GTK_LABEL(overlay_cpu_tail), 1.0f);
    gtk_widget_set_halign(overlay_cpu_tail, GTK_ALIGN_END);
    gtk_widget_set_valign(overlay_cpu_tail, GTK_ALIGN_CENTER);
    gtk_style_context_add_class(gtk_widget_get_style_context(overlay_cpu_tail),
            "ov-sub");
    gtk_box_pack_end(GTK_BOX(hero_top), overlay_cpu_tail, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hero), hero_top, TRUE, TRUE, 0);

    overlay_spark = gtk_drawing_area_new();
    gtk_widget_set_size_request(overlay_spark, -1, 22);
    ui_overlay_pass_through(overlay_spark);
    g_signal_connect(overlay_spark, "draw",
            G_CALLBACK(ui_overlay_spark_draw), NULL);
    gtk_box_pack_start(GTK_BOX(hero), overlay_spark, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(outer), hero, TRUE, TRUE, 0);

    /* ② GPU */
    GtkWidget* row_gpu = ui_overlay_row(GTK_ORIENTATION_HORIZONTAL);
    overlay_ring_gpu = ui_overlay_ring();
    gtk_box_pack_start(GTK_BOX(row_gpu), ui_overlay_cap("GPU"), FALSE, FALSE,
            0);
    gtk_box_pack_start(GTK_BOX(row_gpu), overlay_ring_gpu, FALSE, FALSE, 0);
    overlay_gpu_val = ui_overlay_value("ov-val");
    gtk_box_pack_start(GTK_BOX(row_gpu), overlay_gpu_val, FALSE, FALSE, 0);

    overlay_gpu_tail = gtk_label_new(NULL);
    gtk_label_set_xalign(GTK_LABEL(overlay_gpu_tail), 1.0f);
    gtk_widget_set_halign(overlay_gpu_tail, GTK_ALIGN_END);
    gtk_style_context_add_class(gtk_widget_get_style_context(overlay_gpu_tail),
            "ov-sub");
    gtk_box_pack_end(GTK_BOX(row_gpu), overlay_gpu_tail, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(outer), row_gpu, TRUE, TRUE, 0);

    /* ③ 风扇档位 */
    GtkWidget* row_duty = ui_overlay_row(GTK_ORIENTATION_HORIZONTAL);
    overlay_ring_duty = ui_overlay_ring();
    gtk_box_pack_start(GTK_BOX(row_duty), ui_overlay_cap("风扇"), FALSE, FALSE,
            0);
    gtk_box_pack_start(GTK_BOX(row_duty), overlay_ring_duty, FALSE, FALSE, 0);
    overlay_duty_val = ui_overlay_value("ov-val");
    gtk_box_pack_start(GTK_BOX(row_duty), overlay_duty_val, FALSE, FALSE, 0);

    overlay_fan_mode = gtk_label_new(NULL);
    gtk_label_set_xalign(GTK_LABEL(overlay_fan_mode), 1.0f);
    gtk_widget_set_halign(overlay_fan_mode, GTK_ALIGN_END);
    gtk_style_context_add_class(gtk_widget_get_style_context(overlay_fan_mode),
            "ov-sub");
    gtk_box_pack_end(GTK_BOX(row_duty), overlay_fan_mode, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(outer), row_duty, TRUE, TRUE, 0);

    /* ④ 转速 */
    GtkWidget* row_rpm = ui_overlay_row(GTK_ORIENTATION_HORIZONTAL);
    overlay_ring_rpm = ui_overlay_ring();
    gtk_box_pack_start(GTK_BOX(row_rpm), ui_overlay_cap("转速"), FALSE, FALSE,
            0);
    gtk_box_pack_start(GTK_BOX(row_rpm), overlay_ring_rpm, FALSE, FALSE, 0);
    overlay_rpm_val = ui_overlay_value("ov-val");
    gtk_box_pack_start(GTK_BOX(row_rpm), overlay_rpm_val, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(outer), row_rpm, TRUE, TRUE, 0);

    /* 固定宽度：数值位数变化时卡片不左右伸缩 */
    gtk_widget_set_size_request(outer, 264, -1);

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
    int gt = (int) share_info->gpu_temp;
    int duty = (int) share_info->fan_duty;
    int rpm = (int) share_info->fan_rpms;
    int rpm2 = (int) share_info->fan2_rpms;
    int auto_mode = share_info->auto_duty ? 1 : 0;

    /* ① CPU 大字。字号由 CSS 决定，这里只覆盖颜色 */
    snprintf(buf, sizeof(buf),
            "<span foreground='%s'>%d°</span>",
            ui_overlay_temp_color(t), t);
    gtk_label_set_markup(GTK_LABEL(overlay_cpu_val), buf);

    /* 主指标行右侧：CPU 电源模式。
     * 这里原来重复显示「档位% · 自动/手动」，和下面「风扇」行完全同义，
     * 换成 EPP 档位（省电 / 系统默认）后才有独立信息量。*/
    int epp = share_info->epp_state;
    snprintf(buf, sizeof(buf),
            "<span size='small' foreground='%s'>●</span>"
            "<span foreground='%s'> %s</span>",
            epp == EPP_POWER ? "#7cc48a" : "#8a8d94",
            epp == EPP_POWER ? "#a8d4b2" : "#9a9da4",
            epp == EPP_POWER ? "省电" : "系统默认");
    gtk_label_set_markup(GTK_LABEL(overlay_cpu_tail), buf);

    /* ② GPU 温度：本机 EC 不上报二极管，NVML 也拿不到时显示 -- */
    if (gt > 0) {
        snprintf(buf, sizeof(buf),
                "<span foreground='%s'>%d°</span>",
                ui_overlay_temp_color(gt), gt);
        ui_overlay_ring_set(overlay_ring_gpu,
                (gt - 20) / 75.0, ui_overlay_ring_temp_color(gt));
    } else {
        snprintf(buf, sizeof(buf),
                "<span foreground='#6b6e75'>--</span>");
        ui_overlay_ring_set(overlay_ring_gpu, 0.0, ui_overlay_ring_temp_color(0));
    }
    gtk_label_set_markup(GTK_LABEL(overlay_gpu_val), buf);

    if (rpm2 > 0) {
        snprintf(buf, sizeof(buf),
                "<span foreground='#878a91'>%d RPM</span>",
                rpm2);
        gtk_label_set_markup(GTK_LABEL(overlay_gpu_tail), buf);
    } else {
        /* 单风扇机型没有 GPU 风扇，右侧留空而不是显示 0 */
        gtk_label_set_text(GTK_LABEL(overlay_gpu_tail), "");
    }

    /* ③ 档位 */
    snprintf(buf, sizeof(buf),
            "<span foreground='#8ab4f8'>%d%%</span>", duty);
    gtk_label_set_markup(GTK_LABEL(overlay_duty_val), buf);
    ui_overlay_ring_set(overlay_ring_duty, duty / 100.0, "#8ab4f8");

    gtk_label_set_markup(GTK_LABEL(overlay_fan_mode),
            auto_mode
                    ? "<span foreground='#878a91'>自动</span>"
                    : "<span foreground='#d9a066'>手动</span>");

    /* ④ 双风扇转速 */
    if (rpm2 > 0) {
        snprintf(buf, sizeof(buf),
                "<span foreground='#eef1f4'>%d</span>"
                "<span foreground='#6b6e75'> / </span>"
                "<span foreground='#eef1f4'>%d</span>",
                rpm, rpm2);
    } else {
        snprintf(buf, sizeof(buf),
                "<span foreground='#eef1f4'>%d</span>", rpm);
    }
    gtk_label_set_markup(GTK_LABEL(overlay_rpm_val), buf);
    int peak = MAX(rpm, rpm2);
    ui_overlay_ring_set(overlay_ring_rpm,
            (double) peak / (double) MAX_FAN_RPM, "#8ab4f8");


    /* 温度走势：UI 每 500ms 刷一次，这里每两次采一个点，即每秒一个 */
    if (++overlay_spark_tick >= 2) {
        overlay_spark_tick = 0;
        overlay_spark_hist[overlay_spark_pos] = t;
        overlay_spark_pos = (overlay_spark_pos + 1) % OVERLAY_SPARK_POINTS;
        if (overlay_spark_len < OVERLAY_SPARK_POINTS)
            overlay_spark_len++;
        if (overlay_spark != NULL)
            gtk_widget_queue_draw(overlay_spark);
    }
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

/* 读取失败返回 -1（区别于「确实是默认档」），否则返回 EPP_POWER / EPP_DEFAULT */
static int sysfs_query_epp(void) {
    char buf[64] = {0};
    int fd = open(
            "/sys/devices/system/cpu/cpufreq/policy0/energy_performance_preference",
            O_RDONLY);
    if (fd < 0)
        return -1;
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0)
        return -1;
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

/* ================= 风扇曲线 =================
 *
 * 参考 NBFC-Linux / thinkfan / ClevoFanControl / asus-wmi 的成熟做法：
 *
 *   1. 曲线是一张「温度 : 占空比」节点表，节点之间线性插值
 *      （可用 mode step 改成阶梯跳变，行为等同 asus-wmi 的 10°C 阶梯）
 *   2. CPU、GPU 各一条曲线，每轮各算一次，取需求大的一方 ——
 *      风扇是两热源共用的，谁更热就按谁吹
 *   3. 升温用瞬时温度（保护硬件），降温用 6 秒平均值，并且必须低于
 *      「当前档位的入口温度 - hyst」才降，这就是 NBFC 的 Up/DownThreshold
 *   4. 单步升降有幅度上限，避免温度尖峰把风扇瞬间拉满、再瞬间落回
 *   5. 临界温度立即全速，并锁存到回落到 critical - critical_offset
 *      以下才交还曲线（NBFC 的 CriticalTemperature 机制）
 *
 * 节点表可以在 ~/.config/clevo-indicator/curve 里覆盖，格式见 README。
 */

#define CURVE_MAX_NODES 10
#define CURVE_PATH_MAX  512

typedef struct {
    int temp;
    int duty;
} FanNode;

typedef struct {
    FanNode nodes[CURVE_MAX_NODES];
    int count;
} FanCurve;

/* 默认 CPU 曲线按本机实测标定：空闲 46–50°C 落在 22–24%（约 1900–2100 RPM），
 * 60°C 约 34%，70°C 约 50%，80°C 约 73%，89°C 起满速。*/
static FanCurve curve_cpu = {
    .count = 8,
    .nodes = {
        { 40, 18 }, { 48, 22 }, { 56, 28 }, { 64, 40 },
        { 72, 54 }, { 78, 68 }, { 84, 84 }, { 89, 100 }
    }
};

/* GPU 曲线比 CPU 更陡：Max-Q 显卡空闲只有 30–40°C，一旦跑到 75°C 以上
 * 就该大幅加风，因此低频段给得低、高频段给得狠，避免平时白吹。*/
static FanCurve curve_gpu = {
    .count = 8,
    .nodes = {
        { 40, 18 }, { 55, 22 }, { 62, 30 }, { 70, 45 },
        { 76, 60 }, { 80, 75 }, { 85, 90 }, { 88, 100 }
    }
};

static int curve_hyst = 3;              /* 降温滞回，°C */
static int curve_critical = 92;         /* 临界温度，强制 100% */
static int curve_critical_off = 6;      /* 回落这么多才退出强制全速 */
static int curve_ramp_up = 20;          /* 单步最多升多少 %（200ms 一步） */
static int curve_ramp_down = 3;         /* 单步最多降多少 % */

/* 占空比死区：比当前高出 4% 才升、低出 2% 才降。
 * 「升的阈值必须大于降的单步幅度」，否则降完一档立刻又被判为「该升了」，
 * 会在同一个温度上来回写 EC（实测 25%→22%→24% 连写三次）。
 * 4% 的代价是稳定时比理想值低最多 4%（约 2°C 更热），换取安静。*/
#define CURVE_DEADBAND_UP   4
#define CURVE_DEADBAND_DOWN 2
static int curve_linear = 1;            /* 1=线性插值，0=阶梯跳变 */
static int curve_critical_latch = 0;
static char curve_last_reason[64] = "";
static char curve_config_path[CURVE_PATH_MAX] = "";

static const char* curve_reason(void) {
    return curve_last_reason;
}

/* 曲线在某温度处给出的占空比 */
static int curve_eval(const FanCurve* c, int temp) {
    if (c->count <= 0)
        return FAN_AUTO_MIN;
    if (temp <= c->nodes[0].temp)
        return c->nodes[0].duty;
    for (int i = 0; i + 1 < c->count; i++) {
        const FanNode* a = &c->nodes[i];
        const FanNode* b = &c->nodes[i + 1];
        if (temp < b->temp) {
            if (!curve_linear || a->temp == b->temp)
                return a->duty;
            return a->duty
                    + (temp - a->temp) * (b->duty - a->duty)
                            / (b->temp - a->temp);
        }
    }
    return c->nodes[c->count - 1].duty;
}

/* 反向：曲线给出这个占空比所需的入口温度。
 * 降温时要求「平均值低于入口温度 - hyst」才降档，等价于 NBFC 的
 * DownThreshold，比单纯按占空比做死区更贴合曲线形状。*/
static int curve_entry_temp(const FanCurve* c, int duty) {
    if (c->count <= 0)
        return curve_critical;
    if (duty <= c->nodes[0].duty)
        return c->nodes[0].temp;
    for (int i = 0; i + 1 < c->count; i++) {
        const FanNode* a = &c->nodes[i];
        const FanNode* b = &c->nodes[i + 1];
        if (duty >= a->duty && duty <= b->duty) {
            if (!curve_linear || b->duty == a->duty)
                return b->temp;
            return a->temp
                    + (duty - a->duty) * (b->temp - a->temp)
                            / (b->duty - a->duty);
        }
    }
    return c->nodes[c->count - 1].temp;
}

/* 控制温度：升立即跟随、降按 1/8 衰减，且最多只比实测低 MAX_LAG 度。
 *
 * 为什么必须只用一个温度信号：早先的做法是「升温看瞬时值、降温看 6 秒均值」，
 * 两个信号在升温斜坡上必然差 3°C 以上，于是每一轮都是「按瞬时值升档 →
 * 按均值降档」，风扇在 26%/31% 之间来回跳。改成单一信号后方向只有一种。
 *
 * MAX_LAG 的作用是兜住瞬时尖峰：温度从 85 掉回 48 时，如果只靠 1/8 衰减，
 * 控制温度会在 80 附近磨蹭好几秒，风扇被这一拍假数据顶到 75%；
 * 夹住「实测 + 4°C」之后，下一轮就回到 52，1.5 秒内落回原档。*/
#define CURVE_MAX_LAG 4
static int curve_ctrl_t10[2] = { 0, 0 };   /* 控制温度，单位 0.1°C；[0]=CPU [1]=GPU */
static int curve_ctrl_temp[2] = { 0, 0 };
static int curve_driver = 0;              /* 当前由哪个热源主导：0=CPU 1=GPU */

static int curve_filter_temp(int idx, int raw) {
    int x = raw * 10;
    if (x >= curve_ctrl_t10[idx]) {
        curve_ctrl_t10[idx] = x;        /* 升温：立即跟随 */
    } else {
        int gap = curve_ctrl_t10[idx] - x;
        int dec = gap / 8;
        if (dec < 1)
            dec = 1;
        curve_ctrl_t10[idx] -= dec;     /* 降温：约 1.6 秒时间常数 */
        if (curve_ctrl_t10[idx] > x + CURVE_MAX_LAG * 10)
            curve_ctrl_t10[idx] = x + CURVE_MAX_LAG * 10;
    }
    curve_ctrl_temp[idx] = curve_ctrl_t10[idx] / 10;
    return curve_ctrl_temp[idx];
}

static int curve_ctrl_temperature(void) {
    return curve_ctrl_temp[curve_driver];
}

/* 每轮调速的核心：返回目标占空比，0 表示保持现状。
 * 每个热源用各自的温度查各自的曲线，再取需求更大的那个 ——
 * 风扇是两个热源共用的，GPU 打游戏时温度会超过 CPU，不能拿 CPU 的温度
 * 去查 GPU 曲线（那样 GPU 一升温反而按 CPU 曲线算，等于白算）。*/
static int curve_auto_target(int cpu_temp, int gpu_temp) {
    int ctrl_cpu = curve_filter_temp(0, cpu_temp);
    int cpu_need = curve_eval(&curve_cpu, ctrl_cpu);
    int ctrl = ctrl_cpu;
    int need = cpu_need;

    if (gpu_temp > 0) {
        int ctrl_gpu = curve_filter_temp(1, gpu_temp);
        int gpu_need = curve_eval(&curve_gpu, ctrl_gpu);
        if (gpu_need > need) {
            need = gpu_need;
            ctrl = ctrl_gpu;
            curve_driver = 1;
        } else {
            curve_driver = 0;
        }
    } else {
        curve_driver = 0;
    }
    const char* src = curve_driver ? "GPU" : "CPU";

    /* 用「上次写入值」而非 EC 回读值作比较基准：
     * 回读是 0–255 量化后的百分比（写 28% 回读 27%），
     * 用它比较会导致每 200ms 重复写一次 EC。*/
    int cur = share_info->auto_duty_val;
    if (cur <= 0)
        cur = share_info->fan_duty;     /* 刚切到自动模式，以 EC 实际值为起点 */
    if (cur <= 0)
        cur = FAN_AUTO_MIN;

    /* ① 临界温度：用实测值判断（不等滤波），无视曲线直接全速，
     *    并锁存到控制温度回落到 critical - offset 以下才交还曲线 */
    if (cpu_temp >= curve_critical || gpu_temp >= curve_critical) {
        curve_critical_latch = 1;
        snprintf(curve_last_reason, sizeof(curve_last_reason),
                "临界 %d°C 强制全速", MAX(cpu_temp, gpu_temp));
        return 100;
    }
    if (curve_critical_latch) {
        if (ctrl > curve_critical - curve_critical_off) {
            snprintf(curve_last_reason, sizeof(curve_last_reason),
                    "临界锁存（控制温度 %d°C）", ctrl);
            return 100;
        }
        snprintf(curve_last_reason, sizeof(curve_last_reason), "退出临界锁存");
        curve_critical_latch = 0;
    }

    /* ② 升温：立即响应，但单步限制幅度，滤掉短时温度尖峰 */
    if (need >= cur + CURVE_DEADBAND_UP) {
        int target = MIN(need, cur + curve_ramp_up);
        if (target > cur) {
            snprintf(curve_last_reason, sizeof(curve_last_reason),
                    "%s 曲线 %d°C", src, ctrl);
            return target;
        }
    }

    /* ③ 降温：控制温度必须低于「当前档位入口温度 - 滞回」才降，
     *    等价于 NBFC 的 DownThreshold，且单步限幅 */
    const FanCurve* dc = curve_driver ? &curve_gpu : &curve_cpu;
    int entry = curve_entry_temp(dc, cur);
    if (need <= cur - CURVE_DEADBAND_DOWN && ctrl <= entry - curve_hyst) {
        int target = MAX(need, cur - curve_ramp_down);
        if (target < cur) {
            snprintf(curve_last_reason, sizeof(curve_last_reason),
                    "降温（%d°C < 入口 %d°C - %d）",
                    ctrl, entry, curve_hyst);
            return target;
        }
    }

    return 0;
}

/* ---------- 曲线配置加载 ---------- */

static int curve_parse_nodes(FanCurve* c, char* text) {
    FanCurve tmp;
    memset(&tmp, 0, sizeof(tmp));
    char* save = NULL;
    for (char* tok = strtok_r(text, " \t\r\n", &save); tok != NULL;
            tok = strtok_r(NULL, " \t\r\n", &save)) {
        char* colon = strchr(tok, ':');
        if (colon == NULL)
            return -1;
        *colon = '\0';
        int t = atoi(tok);
        int d = atoi(colon + 1);
        if (t < 20 || t > 110 || d < 5 || d > 100)
            return -1;
        if (tmp.count >= CURVE_MAX_NODES)
            return -1;
        if (tmp.count > 0) {
            /* 温度必须递增、占空比不许回落，否则曲线语义就乱套了 */
            if (t <= tmp.nodes[tmp.count - 1].temp)
                return -1;
            if (d < tmp.nodes[tmp.count - 1].duty)
                return -1;
        }
        tmp.nodes[tmp.count].temp = t;
        tmp.nodes[tmp.count].duty = d;
        tmp.count++;
    }
    if (tmp.count < 2)
        return -1;
    *c = tmp;
    return 0;
}

static int curve_clamp(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void curve_load(void) {
    const char* env = getenv("CLEVO_CURVE");
    const char* home = getenv("HOME");
    if (env != NULL && env[0] != '\0') {
        snprintf(curve_config_path, sizeof(curve_config_path), "%s", env);
    } else if (home != NULL && home[0] != '\0') {
        snprintf(curve_config_path, sizeof(curve_config_path),
                "%s/.config/clevo-indicator/curve", home);
    } else {
        return;                         /* 没有 HOME，就用内置默认曲线 */
    }

    FILE* fp = fopen(curve_config_path, "r");
    if (fp == NULL) {
        printf("未找到曲线配置 %s，使用内置默认曲线\n", curve_config_path);
        return;
    }

    char line[512];
    int lineno = 0;
    while (fgets(line, sizeof(line), fp) != NULL) {
        lineno++;
        char* hash = strchr(line, '#');
        if (hash != NULL)
            *hash = '\0';
        char* p = line;
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\0' || *p == '\n' || *p == '\r')
            continue;
        char* key = p;
        char* sep = strpbrk(p, " \t");
        char* rest = (char*) "";
        if (sep != NULL) {
            *sep = '\0';
            rest = sep + 1;
        }
        if (strcmp(key, "cpu") == 0) {
            if (curve_parse_nodes(&curve_cpu, rest) != 0) {
                printf("曲线配置第 %d 行无效（温度需递增、占空比不可回落），"
                       "CPU 曲线保留默认\n", lineno);
            }
        } else if (strcmp(key, "gpu") == 0) {
            if (curve_parse_nodes(&curve_gpu, rest) != 0) {
                printf("曲线配置第 %d 行无效，GPU 曲线保留默认\n", lineno);
            }
        } else if (strcmp(key, "hyst") == 0) {
            curve_hyst = curve_clamp(atoi(rest), 0, 15);
        } else if (strcmp(key, "critical") == 0) {
            curve_critical = curve_clamp(atoi(rest), 60, 105);
        } else if (strcmp(key, "critical_offset") == 0) {
            curve_critical_off = curve_clamp(atoi(rest), 0, 25);
        } else if (strcmp(key, "ramp_up") == 0) {
            curve_ramp_up = curve_clamp(atoi(rest), 1, 100);
        } else if (strcmp(key, "ramp_down") == 0) {
            curve_ramp_down = curve_clamp(atoi(rest), 1, 100);
        } else if (strcmp(key, "mode") == 0) {
            if (strncmp(rest, "step", 4) == 0)
                curve_linear = 0;
            else if (strncmp(rest, "linear", 6) == 0)
                curve_linear = 1;
        } else {
            printf("曲线配置第 %d 行：未知配置项 %s，已忽略\n", lineno, key);
        }
    }
    fclose(fp);
    printf("已加载曲线配置 %s（%s，滞回 %d°C，临界 %d°C，"
           "上升 +%d%%/步，下降 -%d%%/步）\n",
            curve_config_path, curve_linear ? "线性插值" : "阶梯跳变",
            curve_hyst, curve_critical, curve_ramp_up, curve_ramp_down);
}

/* --curve：把当前曲线（含插值结果）打印成表，方便验证配置改得对不对 */
static int main_print_curve(void) {
    printf("曲线配置：%s\n", curve_config_path[0] ? curve_config_path : "内置默认");
    printf("插值方式：%s   滞回：%d°C   临界：%d°C（回落 %d°C 解除）\n",
            curve_linear ? "线性" : "阶梯", curve_hyst, curve_critical,
            curve_critical_off);
    printf("单步限幅：上升 +%d%%  下降 -%d%%\n\n", curve_ramp_up,
            curve_ramp_down);
    printf(" 温度   CPU曲线   GPU曲线   同温取用\n");
    printf(" ------------------------------------\n");
    for (int t = 30; t <= 100; t += 2) {
        int a = curve_eval(&curve_cpu, t);
        int b = curve_eval(&curve_gpu, t);
        int use = MAX(a, b);
        printf(" %3d°C  %5d%%   %5d%%    %4d%%%s\n", t, a, b, use,
                (t >= curve_critical) ? "  ← 临界全速" : "");
    }
    printf("\n注：运行时 CPU、GPU 各查各的曲线再取较大者，上表假设两者同温。\n");
    return EXIT_SUCCESS;
}

static int ec_query_cpu_temp(void) {
    return ec_io_read(EC_REG_CPU_TEMP);
}

static int ec_query_gpu_temp(void) {
    return ec_io_read(EC_REG_GPU_TEMP);
}

/* ---------- GPU 温度（NVIDIA 驱动接口） ----------
 * 本机 EC 的 0xCD 恒为 0：混合显卡（Optimus）机型上 EC 固件不映射 GPU
 * 二极管，只有独显全权接管时才回值。NVML 可以直接向驱动要温度，
 * 用 dlopen 按需加载，没装 NVIDIA 驱动时安静退回 EC 值。*/

typedef int nvml_return_t;
typedef void* nvml_device_t;

#define NVML_TEMPERATURE_GPU 0

static void* nvml_lib = NULL;
static nvml_device_t nvml_dev = NULL;
static nvml_return_t (*nvml_get_temp)(nvml_device_t, int, unsigned int*) = NULL;
static int nvml_state = 0;   /* 0 = 未尝试, 1 = 可用, -1 = 不可用 */

static int nvml_init(void) {
    if (nvml_state != 0)
        return nvml_state > 0;

    nvml_state = -1;
    void* lib = dlopen("libnvidia-ml.so.1", RTLD_NOW | RTLD_LOCAL);
    if (lib == NULL)
        lib = dlopen("/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1",
                RTLD_NOW | RTLD_LOCAL);
    if (lib == NULL) {
        printf("NVML 不可用（没装 NVIDIA 驱动？），GPU 温度回退 EC\n");
        return 0;
    }
    nvml_return_t (*fn_init)(void) = dlsym(lib, "nvmlInit_v2");
    nvml_return_t (*fn_count)(unsigned int*) = dlsym(lib,
            "nvmlDeviceGetCount_v2");
    nvml_return_t (*fn_handle)(unsigned int, nvml_device_t*) = dlsym(lib,
            "nvmlDeviceGetHandleByIndex_v2");
    nvml_get_temp = dlsym(lib, "nvmlDeviceGetTemperature");
    if (fn_init == NULL || fn_count == NULL || fn_handle == NULL
            || nvml_get_temp == NULL) {
        dlclose(lib);
        printf("NVML 符号缺失，GPU 温度回退 EC\n");
        return 0;
    }
    unsigned int n = 0;
    if (fn_init() != 0 || fn_count(&n) != 0 || n == 0
            || fn_handle(0, &nvml_dev) != 0) {
        dlclose(lib);
        printf("NVML 初始化失败，GPU 温度回退 EC\n");
        return 0;
    }
    nvml_lib = lib;
    nvml_state = 1;
    printf("NVML 已就绪，GPU 温度由驱动提供（%u 个设备）\n", n);
    return 1;
}

static int nvml_gpu_temperature(void) {
    if (!nvml_init())
        return -1;
    unsigned int t = 0;
    if (nvml_get_temp(nvml_dev, NVML_TEMPERATURE_GPU, &t) != 0)
        return -1;
    return (t > 0 && t < 150) ? (int) t : -1;
}

/* 统一入口：优先 EC（全权独显机型有真实值），无效时问驱动 */
static int gpu_temperature_any(void) {
    int t = ec_query_gpu_temp();
    if (t > 0 && t < 110)
        return t;
    return nvml_gpu_temperature();
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

/* 记录发生 EPP 变化时的调频上下文，便于回溯是谁动的 */
static void ec_freq_context(char* buf, size_t size) {
    char gov[32] = "?";
    int freq = 0;
    int fd = open("/sys/devices/system/cpu/cpufreq/policy0/scaling_governor",
            O_RDONLY);
    if (fd >= 0) {
        ssize_t n = read(fd, gov, sizeof(gov) - 1);
        if (n > 0)
            gov[n - 1] = '\0';
        close(fd);
    }
    fd = open("/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq",
            O_RDONLY);
    if (fd >= 0) {
        char b[32] = { 0 };
        if (read(fd, b, sizeof(b) - 1) > 0)
            freq = atoi(b) / 1000;
        close(fd);
    }
    snprintf(buf, size, "governor=%s 当前=%dMHz", gov, freq);
}

/* worker 的日志统一走这里。
 * 注意：worker 已经 setuid(0)，它的 stdout 归到 root 名下，所以
 * `journalctl --user` 看不到，要用系统日志查：
 *     sudo journalctl -t clevo-indicator
 * 之前误以为 fork 后日志会丢，其实是查错了地方；加 vsyslog 只会让每条
 * 重复两遍，所以这里只保留 stdout。*/
static void ec_log(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
    vprintf(format, ap);
    va_end(ap);
    fflush(stdout);
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
