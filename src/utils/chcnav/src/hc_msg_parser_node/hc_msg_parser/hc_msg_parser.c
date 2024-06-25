#include "hc_msg_parser.h"

#include <assert.h>
#include <unistd.h>

/**
 * @brief reset a token object
 *
 * @param token a token object
 * */
void hc__msg_reset_token(hc__msg_token_t *token)
{
    assert(token); /* Non-NULL token object expected. */

    memset(token, 0, sizeof(hc__msg_token_t));
}

/**
 * @brief initialize a parser
 *
 * @param parser
 * @return int @c 0 if the function succeeded, @c others on error.
 * */
int hc__init_msg_parser(hc__msg_parser_t *parser, size_t buffer_size, hc__msg_read_handler_f read_handler, void *parser_id)
{
    assert(parser);      /* Non-NULL token object expected. */
    assert(buffer_size); /* None 0 expected. */

    parser->error.error_code = HC__STATE_NONE;
    strncpy(parser->error.description, "No error", 9);

    parser->state = HC__STATE_S0;

    parser->buffer.end = buffer_size;
    parser->buffer.pointer = 0;
    parser->buffer.last = 0;
    parser->buffer.value = (void *)malloc(buffer_size);
    parser->parser_id = parser_id;
    parser->read_handler = read_handler;

    return 0;
}

/**
 * Functions to deal all state
 * return value means pointer move steps
 * */
static int hc__msg_deal_S0(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S1(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S2(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S3(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S4(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S5(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S6(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S7(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S8(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S9(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S10(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S11(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S12(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S13(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S14(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S15(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S16(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S17(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S18(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S19(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S20(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S21(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S22(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S23(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S24(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);

// bind state with proc func
static int (*g_hc__msg_state_handler[])(hc__msg_parser_t *, hc__msg_token_t *, unsigned char) = {
    [HC__STATE_S0] = hc__msg_deal_S0,
    [HC__STATE_S1] = hc__msg_deal_S1,
    [HC__STATE_S2] = hc__msg_deal_S2,
    [HC__STATE_S3] = hc__msg_deal_S3,
    [HC__STATE_S4] = hc__msg_deal_S4,
    [HC__STATE_S5] = hc__msg_deal_S5,
    [HC__STATE_S6] = hc__msg_deal_S6,
    [HC__STATE_S7] = hc__msg_deal_S7,
    [HC__STATE_S8] = hc__msg_deal_S8,
    [HC__STATE_S9] = hc__msg_deal_S9,
    [HC__STATE_S10] = hc__msg_deal_S10,
    [HC__STATE_S11] = hc__msg_deal_S11,
    [HC__STATE_S12] = hc__msg_deal_S12,
    [HC__STATE_S13] = hc__msg_deal_S13,
    [HC__STATE_S14] = hc__msg_deal_S14,
    [HC__STATE_S15] = hc__msg_deal_S15,
    [HC__STATE_S16] = hc__msg_deal_S16,
    [HC__STATE_S17] = hc__msg_deal_S17,
    [HC__STATE_S18] = hc__msg_deal_S18,
    [HC__STATE_S19] = hc__msg_deal_S19,
    [HC__STATE_S20] = hc__msg_deal_S20,
    [HC__STATE_S21] = hc__msg_deal_S21,
    [HC__STATE_S22] = hc__msg_deal_S22,
    [HC__STATE_S23] = hc__msg_deal_S23,
    [HC__STATE_S24] = hc__msg_deal_S24,
};

/**
 * Scan the input stream and produce the next token.
 *
 * Call the function subsequently to produce a sequence of tokens corresponding
 * to the input stream.
 *
 * An application is responsible for freeing any buffers associated with the
 * produced token object using the @c hsl__file_token_delete function.
 *
 * @param[in,out]   parser      A parser object.
 * @param[out]      token       An token object.
 *
 * @returns @c 0 if the function succeeded, @c -1 on error.
 */

int hc__msg_parser_scan(hc__msg_parser_t *parser, hc__msg_token_t *token)
{
    assert(parser); /* Non-NULL parser object is expected. */
    assert(token);  /* Non-NULL token object is expected. */

    hc__msg_reset_token(token);

    if (parser->error.error_code != HC__STATE_NONE)
        return -1;

    while (1)
    {
        if (parser->buffer.pointer == parser->buffer.last)
        {
            memset((void *)parser->buffer.value, 0x00, parser->buffer.end);
            parser->read_handler((void *)parser->buffer.value, parser->buffer.end, &parser->buffer.last, parser->parser_id);
            parser->buffer.pointer = 0;
            // wait until at least one char read
            while (parser->buffer.last <= 0)
            {
                parser->read_handler((void *)parser->buffer.value, parser->buffer.end, &parser->buffer.last, parser->parser_id);
                usleep(1000);
            }  
        }

        unsigned char c = ((unsigned char *)parser->buffer.value)[parser->buffer.pointer];

        // deal all state. call func pointer
        if (parser->state <= HC__STATE_S24 && parser->state >= HC__STATE_S0)
        {
            int pointer_move_step = g_hc__msg_state_handler[parser->state](parser, token, c);

            int new_token_value_len = token->token_value.value_len + pointer_move_step;
            int new_token_pointer = parser->buffer.pointer + pointer_move_step;

            if (new_token_value_len < sizeof(token->token_value.value))
            {
                memcpy(token->token_value.value + token->token_value.value_len, parser->buffer.value + parser->buffer.pointer, pointer_move_step);
                token->token_value.value_len = new_token_value_len;
                parser->buffer.pointer = new_token_pointer;
            }
            else
            {
                token->type = HC__TOKERN_ERROR;
                parser->error.error_code = parser->state;
                snprintf(parser->error.description, sizeof(parser->error.description), "token value too long [%d]", token->token_value.value_len);
            }
        }
        else
        {
            fprintf(stderr, "state S[%d]", parser->state);
            assert(0);
        }

        if (token->type != HC__TOKERN_NONE)
            break;
    }

    return 0;
}

/* *******************************************************************
 * Functions dealing state
 * ***************************************************************** */

static int hc__msg_deal_S0(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0xaa == c)
    {
        parser->state = HC__STATE_S1;
    }
    else if ('$' == c)
    {
        parser->state = HC__STATE_S18;
        parser->specified_len = 5;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S0;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);
    }

    return 1;
}

static int hc__msg_deal_S1(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0xcc == c)
    {
        parser->state = HC__STATE_S2;
    }
    else if (0x55 == c)
    {
        parser->state = HC__STATE_S12;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S1;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S2(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x48 == c)
    {
        parser->state = HC__STATE_S3;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S2;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S3(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x43 == c)
    {
        parser->state = HC__STATE_S4;
        parser->specified_len = 2;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S3;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S4(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S5;
        parser->specified_len = 2;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S5(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S6;
        parser->specified_len = 2;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S6(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S7;
        parser->specified_len = 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S7(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S8;
        parser->specified_len = 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S8(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S9;
        parser->specified_len = 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S9(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S10;
        unsigned char hc_msg_len_l = token->token_value.value[4];
        unsigned char hc_msg_len_h = token->token_value.value[5];
        parser->specified_len = (hc_msg_len_h << 8) + hc_msg_len_l + 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S10(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {      
        parser->specified_len--;
    }

    if (parser->specified_len == 0)
    {
        token->type = HC__TOKERN_HC_MSG;
        parser->state = HC__STATE_S0;
    }

    return 1;
}

static int hc__msg_deal_S11(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    token->type = HC__TOKERN_HC_MSG;
    parser->state = HC__STATE_S0;
    return 0;
}

static int hc__msg_deal_S12(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x48 == c)
    {
        parser->state = HC__STATE_S13;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S12;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S13(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x43 == c)
    {
        parser->state = HC__STATE_S14;
        parser->specified_len = 2;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S13;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S14(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S15;
        parser->specified_len = 2;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S15(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S16;
        unsigned char hc_msg_len_l = token->token_value.value[4];
        unsigned char hc_msg_len_h = token->token_value.value[5];
        parser->specified_len = (hc_msg_len_h << 8) + hc_msg_len_l + 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S16(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }

    if (parser->specified_len == 0)
    {
        token->type = HC__TOKERN_HC_SHORT_MSG;
        parser->state = HC__STATE_S0;
    }

    return 1;
}

static int hc__msg_deal_S17(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    token->type = HC__TOKERN_HC_SHORT_MSG;
    parser->state = HC__STATE_S0;

    return 0;
}

static int hc__msg_deal_S18(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S19;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S19(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    char nmea_head_2[2];

    nmea_head_2[0] = token->token_value.value[1];
    nmea_head_2[1] = token->token_value.value[2];

    if (strcmp(nmea_head_2, "GP") == 0 || strcmp(nmea_head_2, "GN") == 0 || strcmp(nmea_head_2, "BD") == 0 || strcmp(nmea_head_2, "GT") == 0)
    {
        if (',' == c)
            parser->state = HC__STATE_S21;
        else
            parser->state = HC__STATE_S20;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S19;
        snprintf(parser->error.description, sizeof(parser->error.description), "unsupport nmea head [$%s]", nmea_head_2);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S20(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (',' == c)
    {
        parser->state = HC__STATE_S21;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S20;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S21(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if ('*' == c)
    {
        parser->state = HC__STATE_S22;
        parser->specified_len = 4;
    }
    else if ((c <= 'Z' && c >= 'A') || (c <= 'z' && c >= 'a') || (c <= '9' && c >= '0') || (',' == c) || ('.' == c) || ('-' == c))
    {
        return 1;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S21;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S22(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }

    if (parser->specified_len == 0)
    {
        token->type = HC__TOKERN_NMEA_MGS;
        parser->state = HC__STATE_S0;
    }

    return 1;
}

static int hc__msg_deal_S23(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    char nmea_end_2[2];

    nmea_end_2[0] = token->token_value.value[token->token_value.value_len - 2];
    nmea_end_2[1] = token->token_value.value[token->token_value.value_len - 1];

    if (strcmp(nmea_end_2, "\r\n") == 0)
    {
        parser->state = HC__STATE_S24;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S23;
        snprintf(parser->error.description, sizeof(parser->error.description), "error nmea end [0x%x 0x%x]", nmea_end_2[0], nmea_end_2[1]);

        return 0;
    }

    return 0;
}

static int hc__msg_deal_S24(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    token->type = HC__TOKERN_NMEA_MGS;
    parser->state = HC__STATE_S0;

    return 0;
}