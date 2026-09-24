#ifndef COMMAND_VALIDATION_H
#define COMMAND_VALIDATION_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace lower_controller {
class CommandLineAssembler {
    char line_[100] = {};
    unsigned used_ = 0;
    uint32_t first_us_ = 0;
    bool discard_ = false;
public:
    void DiscardUntilDelimiter() { used_ = 0; discard_ = true; }
    // Returned line remains valid until the next call; max 99 characters + NUL.
    const char *Feed(uint8_t byte, uint32_t now) {
        if (used_ && now-first_us_ >= 250000U) DiscardUntilDelimiter();
        if (byte == '\r' || byte == '\n') {
            if (discard_) { discard_ = false; return nullptr; }
            if (!used_) return nullptr;
            while (used_ && line_[used_-1] == ' ') --used_;
            line_[used_] = 0; used_ = 0;
            return line_[0] ? line_ : nullptr;
        }
        if (discard_) return nullptr;
        if (byte < 32 || byte > 126 || used_ >= sizeof(line_)-1) {
            DiscardUntilDelimiter(); return nullptr;
        }
        if (!used_) first_us_ = now;
        line_[used_++] = char(byte);
        return nullptr;
    }
};
inline bool ParseIntegerList(const char *s, unsigned count, int lo, int hi, int32_t *out=nullptr) {
    for (unsigned i=0; i<count; ++i) {
        bool negative=false;
        if (*s=='-' || *s=='+') negative=(*s++=='-');
        if (*s<'0' || *s>'9') return false;
        int32_t value=0;
        do {
            if (value>100000) return false; // no overflow, all current command ranges < 100000.
            value=value*10+(*s++-'0');
        } while (*s>='0' && *s<='9');
        if (negative) value=-value;
        if (value<lo || value>hi) return false;
        if (out) out[i]=value;
        if (i+1<count) { if (*s++!=',') return false; }
        else if (*s) return false;
    }
    return true;
}
inline bool ValidLegacyCommand(const char *s) {
    const char *const exact[]={"ON","OFF","CA","VA","BEG","END","IVA:BEG","IVA:END",
        "DN","UP","W","S","A","D","Z","E","Q","RPY:ON","RPY:OFF","ACL:ON","ACL:OF","ACL:OFF",
        "DEP:ON","DEP:OFF"};
    for (auto command:exact) if (!strcmp(s,command)) return true;
    if (!strncmp(s,"MOT:",4)) return ParseIntegerList(s+4,4,500,2500);
    if (!strncmp(s,"TES:",4)) return ParseIntegerList(s+4,8,1000,2000);
    if (!strncmp(s,"ANG:",4)) return ParseIntegerList(s+4,1,-180,180);
    if (!strncmp(s,"H:",2)) return ParseIntegerList(s+2,1,0,100000); // legacy command unit 0.1 cm
    return false;
}
}
#endif
