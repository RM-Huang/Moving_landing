#ifndef __HC_MSG_PARSER_H_
#define __HC_MSG_PARSER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/cdefs.h>

__BEGIN_DECLS

#ifndef HC_PROTOCOL_MAX_LEN // 此宏定义为解析的所有数据协议中最长的长度
#define HC_PROTOCOL_MAX_LEN 1024
#endif

/**
 * The prototype of a read handler.
 *
 * The read handler is called when the parser needs to read more bytes from the
 * source.  The handler should write not more than @a size bytes to the @a
 * buffer.  The number of written bytes should be set to the @a length variable.
 *
 * @param[in,out]   buffer      A pointer to an application data specified by
 *                              yaml_parser_set_input().
 * @param[in]       size        The size to read.
 * @param[in]       parser_id   A void * pointer to identify parser. To support object orientation
 * @param[out]      size_read   The actual number of bytes read from the source.
 * @returns On success, the handler should return @c 0.  If the handler failed,
 * the returned value should be @c -1.  On EOF, the handler should set the
 * @a size_read to @c 0 and return @c 0.
 * */
typedef int hc__msg_read_handler_f(void *buffer, size_t size, size_t *size_read, void *parser_id);

typedef enum _hc__msg_token_type_e
{
    /* NO TOKEN */
    HC__TOKERN_NONE,
    /* error token means format error */
    HC__TOKERN_ERROR,
    /* msg header msg token */
    HC__TOKERN_HC_MSG,
    /* hc short header msg token */
    HC__TOKERN_HC_SHORT_MSG,
    /* NMEA msg token */
    HC__TOKERN_NMEA_MGS
} hc__msg_token_type_e;

typedef enum hc__msg_parser_state_e
{
    HC__STATE_S0 = 0,
    HC__STATE_S1,
    HC__STATE_S2,
    HC__STATE_S3,
    HC__STATE_S4,
    HC__STATE_S5,
    HC__STATE_S6,
    HC__STATE_S7,
    HC__STATE_S8,
    HC__STATE_S9,
    HC__STATE_S10,
    HC__STATE_S11,
    HC__STATE_S12,
    HC__STATE_S13,
    HC__STATE_S14,
    HC__STATE_S15,
    HC__STATE_S16,
    HC__STATE_S17,
    HC__STATE_S18,
    HC__STATE_S19,
    HC__STATE_S20,
    HC__STATE_S21,
    HC__STATE_S22,
    HC__STATE_S23,
    HC__STATE_S24,
    HC__STATE_NONE
} hc__msg_parser_state_e;

typedef struct _hc__msg_parser_t
{
    /* error handling */
    struct
    {
        /* error state */
        hc__msg_parser_state_e error_code;
        /* error description */
        char description[1024];
    } error;

    hc__msg_parser_state_e state;

    /** The working buffer. */
    struct
    {
        /** The size of the buffer. */
        size_t end;
        /** The current position of the buffer. */
        size_t pointer;
        /** The last filled position of the buffer. */
        size_t last;

        /** buffer */
        void *value;
    } buffer;

    /** Read handler. */
    hc__msg_read_handler_f *read_handler;

    /* parser id */
    void *parser_id;

    /* Specified read length */
    unsigned int specified_len;
} hc__msg_parser_t;

typedef struct _hc__msg_token_t
{
    /* token type */
    hc__msg_token_type_e type;

    /** token value */
    struct
    {
        char value[HC_PROTOCOL_MAX_LEN];
        unsigned int value_len;
    } token_value;
} hc__msg_token_t;

/**
 * @brief reset a token object
 *
 * @param token a token object
 * */
void hc__msg_reset_token(hc__msg_token_t *token);

/**
 * @brief initialize a parser
 *
 * @param parser
 * @return int @c 0 if the function succeeded, @c others on error.
 * */
int hc__init_msg_parser(hc__msg_parser_t *parser, size_t buffer_size, hc__msg_read_handler_f read_handler, void *parser_id);

/**
 * Scan the input stream and produce the next token.
 *
 * Call the function subsequently to produce a sequence of tokens corresponding
 * to the input stream.
 *
 * An application is responsible for freeing any buffers associated with the
 * produced token object using the @c hc__msg_token_delete function.
 *
 * @param[in,out]   parser      A parser object.
 * @param[out]      token       An token object.
 *
 * @returns @c 0 if the function succeeded, @c -1 on error.
 */
int hc__msg_parser_scan(hc__msg_parser_t *parser, hc__msg_token_t *token);

__END_DECLS

#endif
