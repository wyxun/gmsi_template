/**
 * @file   app_plugins.c
 * @brief  Project Custom Plugins Dispatcher for grblHAL
 */

#include "grbl.h"
#include "hal.h"
#include "task.h"
#include "report.h"

// 1. Declare custom user plugins init functions here:
// extern void custom_safety_init(void);

/**
 * @brief Override grblHAL weak function my_plugin_init()
 *        This is called by the core grblHAL library after hardware is set up,
 *        but before entering the main loop.
 */
void my_plugin_init(void)
{
    // 2. Call custom plugin initializers:
    // custom_safety_init();
    
    // 3. Report loaded plugins:
    // report_plugin("APP_PLUGINS", "1.0");
}
