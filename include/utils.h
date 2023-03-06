#ifndef UTIL_H
#define UTIL_H

#define MAKE_NOCOPY(clazz)          \
private:                            \
    clazz(const clazz &o) = delete; \
    clazz &operator=(const clazz &o) = delete;

#define MAKE_NOMOVE(clazz)           \
private:                             \
    clazz(const clazz &&o) = delete; \
    clazz &operator=(const clazz &&o) = delete;

#endif