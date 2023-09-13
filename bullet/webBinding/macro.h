
#ifndef CC_BULLET_MACRO_H
#define CC_BULLET_MACRO_H

#define DLL_EXPORT __attribute__((visibility("default")))
#define DLL_IMPORT(name) __attribute__((__import_module__("env"), __import_name__(#name)))

#endif
