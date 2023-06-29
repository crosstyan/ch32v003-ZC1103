/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_SIMPLE_PB_H_INCLUDED
#define PB_SIMPLE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _Simple {
    int32_t counter;
    pb_callback_t message;
    bool is_red;
    bool is_green;
    bool is_blue;
} Simple;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define Simple_init_default                      {0, {{NULL}, NULL}, 0, 0, 0}
#define Simple_init_zero                         {0, {{NULL}, NULL}, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define Simple_counter_tag                       1
#define Simple_message_tag                       2
#define Simple_is_red_tag                        3
#define Simple_is_green_tag                      4
#define Simple_is_blue_tag                       5

/* Struct field encoding specification for nanopb */
#define Simple_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    counter,           1) \
X(a, CALLBACK, SINGULAR, STRING,   message,           2) \
X(a, STATIC,   SINGULAR, BOOL,     is_red,            3) \
X(a, STATIC,   SINGULAR, BOOL,     is_green,          4) \
X(a, STATIC,   SINGULAR, BOOL,     is_blue,           5)
#define Simple_CALLBACK pb_default_field_callback
#define Simple_DEFAULT NULL

extern const pb_msgdesc_t Simple_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Simple_fields &Simple_msg

/* Maximum encoded size of messages (where known) */
/* Simple_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
