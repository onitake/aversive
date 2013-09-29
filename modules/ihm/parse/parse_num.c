#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <string.h>

#include "parse.h"
#include "parse_num.h"

//#define debug_printf(args...) printf(args)
#define debug_printf(args...) do {} while(0)

/* XXX to remove ?? */
#define U08_MIN 0x00
#define U08_MAX 0xFF
#define U16_MIN 0x0000
#define U16_MAX 0xFFFF
#define U32_MIN 0x00000000
#define U32_MAX 0xFFFFFFFF
#define U64_MIN 0x0000000000000000
#define U64_MAX 0xFFFFFFFFFFFFFFFF
#define S08_MIN 0x80
#define S08_MAX 0x7F
#define S16_MIN 0x8000
#define S16_MAX 0x7FFF
#define S32_MIN 0x80000000
#define S32_MAX 0x7FFFFFFF
#define S64_MIN 0x8000000000000000
#define S64_MAX 0x7FFFFFFFFFFFFFFF


struct token_ops token_num_ops = {
	.parse = parse_num,
	.complete_get_nb = NULL,
	.complete_get_elt = NULL,
	.get_help = get_help_num,
};


enum num_parse_state_t {
	START,
	DEC_NEG,
	BIN,
	HEX,
	FLOAT_POS,
	FLOAT_NEG,
	ERROR,

	FIRST_OK, /* not used */
	ZERO_OK,
	HEX_OK,
	OCTAL_OK,
	BIN_OK,
	DEC_NEG_OK,
	DEC_POS_OK,
	FLOAT_POS_OK,
	FLOAT_NEG_OK,
};

/* Keep it sync with enum in .h */
static const char PROGMEM help1[] = "UINT8";
static const char PROGMEM help2[] = "UINT16";
static const char PROGMEM help3[] = "UINT32";
static const char PROGMEM help4[] = "UINT64";
static const char PROGMEM help5[] = "INT8";
static const char PROGMEM help6[] = "INT16";
static const char PROGMEM help7[] = "INT32";
static const char PROGMEM help8[] = "INT64";
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
static const char PROGMEM help9[] = "FLOAT";
#endif
static PGM_P num_help[] = {
	help1, help2, help3, help4,
	help5, help6, help7, help8,
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
	help9,
#endif
};

static inline int8_t
add_to_res(uint8_t c, uint64_t * res, uint8_t base)
{
	/* overflow */
	if ( (U64_MAX - c) / base < *res ) {
		return -1;
	}

	*res = *res * base + c ;
	return 0;
}


/* parse an int or a float */
int8_t
parse_num(PGM_P tk, const char * srcbuf, void * res)
{
	struct token_num_data nd;
	enum num_parse_state_t st = START;
	const char * buf = srcbuf;
	char c = *buf;
	uint64_t res1=0, res2=0, res3=1;

	memcpy_P(&nd, &((struct token_num *)tk)->num_data, sizeof(nd));

	while ( st != ERROR && c && ! isendoftoken(c) ) {
		debug_printf("%c %x -> ", c, c);
		switch (st) {
		case START:
			if (c == '-') {
				st = DEC_NEG;
			}
			else if (c == '0') {
				st = ZERO_OK;
			}
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
			else if (c == '.') {
				st = FLOAT_POS;
				res1 = 0;
			}
#endif
			else if (c >= '1' && c <= '9') {
				if (add_to_res(c - '0', &res1, 10) < 0)
					st = ERROR;
				else
					st = DEC_POS_OK;
			}
			else  {
				st = ERROR;
			}
			break;

		case ZERO_OK:
			if (c == 'x') {
				st = HEX;
			}
			else if (c == 'b') {
				st = BIN;
			}
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
			else if (c == '.') {
				st = FLOAT_POS;
				res1 = 0;
			}
#endif
			else if (c >= '0' && c <= '7') {
				if (add_to_res(c - '0', &res1, 10) < 0)
					st = ERROR;
				else
					st = OCTAL_OK;
			}			
			else  {
				st = ERROR;
			}
			break;

		case DEC_NEG:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res1, 10) < 0)
					st = ERROR;
				else
					st = DEC_NEG_OK;
			}
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
			else if (c == '.') {
				res1 = 0;
				st = FLOAT_NEG;
			}
#endif
			else {
				st = ERROR;
			}
			break;

		case DEC_NEG_OK:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res1, 10) < 0)
					st = ERROR;
			}
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
			else if (c == '.') {
				st = FLOAT_NEG;
			}
#endif
			else {
				st = ERROR;
			}
			break;

		case DEC_POS_OK:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res1, 10) < 0)
					st = ERROR;
			}
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
			else if (c == '.') {
				st = FLOAT_POS;
			}
#endif
			else {
				st = ERROR;
			}
			break;

		case HEX:
			st = HEX_OK;
			/* no break */
		case HEX_OK:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res1, 16) < 0)
					st = ERROR;
			}
			else if (c >= 'a' && c <= 'f') {
				if (add_to_res(c - 'a' + 10, &res1, 16) < 0)
					st = ERROR;
			}
			else if (c >= 'A' && c <= 'F') {
				if (add_to_res(c - 'A' + 10, &res1, 16) < 0)
					st = ERROR;
			}
			else {
				st = ERROR;
			}
			break;


		case OCTAL_OK:
			if (c >= '0' && c <= '7') {
				if (add_to_res(c - '0', &res1, 8) < 0)
					st = ERROR;
			}
			else {
				st = ERROR;
			}
			break;

		case BIN:
			st = BIN_OK; 
			/* no break */
		case BIN_OK:
			if (c >= '0' && c <= '1') {
				if (add_to_res(c - '0', &res1, 2) < 0)
					st = ERROR;
			}
			else {
				st = ERROR;
			}
			break;

#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
		case FLOAT_POS:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res2, 10) < 0)
					st = ERROR;
				else 
					st = FLOAT_POS_OK;
				res3 = 10;
			}
			else {
				st = ERROR;
			}
			break;

		case FLOAT_NEG:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res2, 10) < 0)
					st = ERROR;
				else 
					st = FLOAT_NEG_OK;
				res3 = 10;
			}
			else {
				st = ERROR;
			}
			break;

		case FLOAT_POS_OK:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res2, 10) < 0)
					st = ERROR;
				if (add_to_res(0, &res3, 10) < 0)
					st = ERROR;
			}
			else {
				st = ERROR;
			}
			break;

		case FLOAT_NEG_OK:
			if (c >= '0' && c <= '9') {
				if (add_to_res(c - '0', &res2, 10) < 0)
					st = ERROR;
				if (add_to_res(0, &res3, 10) < 0)
					st = ERROR;
			}
			else {
				st = ERROR;
			}
			break;
#endif

		default:
			debug_printf("not impl ");
			
		}

		debug_printf("(%d)  (%d)  (%d)\n",
			     (int)res1, (int)res2, (int)res3);

		buf ++;
		c = *buf;

		/* token too long */
		if (buf-srcbuf > 127)
			return -1;
	}
	
	switch (st) {
	case ZERO_OK:
	case DEC_POS_OK:
	case HEX_OK:
	case OCTAL_OK:
	case BIN_OK:
		if ( nd.type == INT8 && res1 <= S08_MAX ) {
			if (res)
				*(int8_t *)res = (int8_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == INT16 && res1 <= S16_MAX ) {
			if (res)
				*(int16_t *)res = (int16_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == INT32 && res1 <= S32_MAX ) {
			if (res)
				*(int32_t *)res = (int32_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == INT64 && res1 <= S64_MAX ) {
			if (res)
				*(int64_t *)res = (int64_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == UINT8 && res1 <= U08_MAX ) {
			if (res)
				*(uint8_t *)res = (uint8_t) res1;
			return (buf-srcbuf);
		}
		else if (nd.type == UINT16  && res1 <= U16_MAX ) {
			if (res)
				*(uint16_t *)res = (uint16_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == UINT32 ) {
			if (res)
				*(uint32_t *)res = (uint32_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == UINT64 ) {
			if (res)
				*(uint64_t *)res = (uint64_t) res1;
			return (buf-srcbuf);
		}
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
		else if ( nd.type == FLOAT ) {
			if (res)
				*(float *)res = (float)res1;
			return (buf-srcbuf);
		}
#endif
		else {
			return -1;
		}
		break;

	case DEC_NEG_OK:
		if ( nd.type == INT8 && res1 <= S08_MAX + 1 ) {
			if (res)
				*(int8_t *)res = - (int8_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == INT16 && res1 <= (uint16_t)S16_MAX + 1 ) {
			if (res)
				*(int16_t *)res = - (int16_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == INT32 && res1 <= (uint32_t)S32_MAX + 1 ) {
			if (res)
				*(int32_t *)res = - (int32_t) res1;
			return (buf-srcbuf);
		}
		else if ( nd.type == INT64 && res1 <= (uint64_t)S64_MAX + 1 ) {
			if (res)
				*(int64_t *)res = - (int64_t) res1;
			return (buf-srcbuf);
		}
#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
		else if ( nd.type == FLOAT ) {
			if (res)
				*(float *)res = - (float)res1;
			return (buf-srcbuf);
		}
#endif
		else {
			return -1;
		}
		break;

#ifndef CONFIG_MODULE_PARSE_NO_FLOAT
	case FLOAT_POS:
	case FLOAT_POS_OK:
		if ( nd.type == FLOAT ) {
			if (res)
				*(float *)res = (float)res1 + ((float)res2 / (float)res3);
			return (buf-srcbuf);
			
		}
		else {
			return -1;
		}
		break;

	case FLOAT_NEG:
	case FLOAT_NEG_OK:
		if ( nd.type == FLOAT ) {
			if (res)
				*(float *)res = - ((float)res1 + ((float)res2 / (float)res3));
			return (buf-srcbuf);
			
		}
		else {
			return -1;
		}
		break;
#endif
	default:
		debug_printf("error\n");
		return -1;
	}
	return -1;
}


/* parse an int or a float */
int8_t
get_help_num(PGM_P tk, char * dstbuf, uint8_t size)
{
	struct token_num_data nd;

	memcpy_P(&nd, &((struct token_num *)tk)->num_data, sizeof(nd));
	
	/* should not happen.... don't so this test */
/* 	if (nd.type >= (sizeof(num_help)/sizeof(const char *))) */
/* 		return -1; */

	strncpy_P(dstbuf, num_help[nd.type], size);
	dstbuf[size-1] = '\0';
	return 0;
}
