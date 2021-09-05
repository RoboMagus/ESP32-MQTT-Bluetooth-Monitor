#ifndef StackDbgHelper_h
#define StackDbgHelper_h

// Basic headers
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stack>

// Tweaked SDK configuration
#include "sdkconfig.h"

// Arduino includes
#include <Arduino.h>

struct stack_entry {
    uint32_t pc;
    const char* func;
    uint32_t line;

    stack_entry(uint32_t _pc, const char* f, uint32_t l) : pc(_pc), func(f), line(l) {

    }
};

extern std::stack<stack_entry> _stack;

class scopedStackEntry {
public:
    scopedStackEntry(uint32_t val, const char* f, uint32_t line) : _entry(val, f, line) {
        _stack.push(_entry);
    }

    ~scopedStackEntry() {
        _stack.pop();
    }

private:
    stack_entry _entry;
};

#define SCOPED_STACK_ENTRY scopedStackEntry _s(_stack.size(), __PRETTY_FUNCTION__, __LINE__);


#endif // StackDbgHelper_h
