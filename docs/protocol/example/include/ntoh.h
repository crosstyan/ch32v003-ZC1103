//
// Created by Kurosu Chan on 2023/6/30.
//

#ifndef SE_DE_TEST_NTOH_H
#define SE_DE_TEST_NTOH_H

#include <arpa/inet.h>

namespace inet {
    inline auto __ntohl(uint32_t x) -> uint32_t {
        return ntohl(x);
    }

    inline auto __ntohs(uint16_t x) -> uint16_t {
        return ntohs(x);
    }

    inline auto __htonl(uint32_t x) -> uint32_t {
        return htonl(x);
    }

    inline auto __htons(uint16_t x) -> uint16_t {
        return htons(x);
    }
}


#endif //SE_DE_TEST_NTOH_H
