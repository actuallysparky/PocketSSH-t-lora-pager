#pragma once

// Boot test hook defaults:
// - Keep this file credential-free for safe commits.
// - Put local secrets in main/include/tpager_test_hook_config_local.hpp
//   (gitignored) using the same macro names below.

#ifndef TPAGER_BOOT_TEST_ENABLE
#define TPAGER_BOOT_TEST_ENABLE 0
#endif

#ifndef TPAGER_BOOT_WIFI_SSID
#define TPAGER_BOOT_WIFI_SSID ""
#endif

#ifndef TPAGER_BOOT_WIFI_PASSWORD
#define TPAGER_BOOT_WIFI_PASSWORD ""
#endif

#ifndef TPAGER_BOOT_SSH_HOST
#define TPAGER_BOOT_SSH_HOST ""
#endif

#ifndef TPAGER_BOOT_SSH_PORT
#define TPAGER_BOOT_SSH_PORT 22
#endif

#ifndef TPAGER_BOOT_SSH_USER
#define TPAGER_BOOT_SSH_USER ""
#endif

#ifndef TPAGER_BOOT_SSH_PASSWORD
#define TPAGER_BOOT_SSH_PASSWORD ""
#endif

#ifndef TPAGER_BOOT_SSH_MDNS_HOST
#define TPAGER_BOOT_SSH_MDNS_HOST ""
#endif
