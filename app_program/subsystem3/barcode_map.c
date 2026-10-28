#include "barcode_map.h"

#include <stddef.h>

#define MAP_MAX_WORDS 80U
#define MAP_MAX_WORD 10U

/* One byte per character. The comms task writes an entry and the S3 task
 * reads it; a single-byte store cannot be seen half-written, so no lock is
 * needed for one entry. A multi-letter message is applied entry by entry,
 * so send it before START, not while the car is reading a barcode. */
static volatile uint8_t map_table[BARCODE_MAP_SIZE] = {
    (uint8_t)ROUTE_COMMAND_LEFT,      /* A */
    (uint8_t)ROUTE_COMMAND_RIGHT,     /* B */
    (uint8_t)ROUTE_COMMAND_STRAIGHT,  /* C */
    (uint8_t)ROUTE_COMMAND_UTURN      /* D, the rest start as NONE (0) */
};

static char upper(char c)
{
    return ((c >= 'a') && (c <= 'z')) ? (char)(c - 'a' + 'A') : c;
}

/* 'A'-'Z' -> 0-25, '0'-'9' -> 26-35, anything else -> -1. */
static int index_of(char c)
{
    c = upper(c);
    if ((c >= 'A') && (c <= 'Z'))
    {
        return c - 'A';
    }
    if ((c >= '0') && (c <= '9'))
    {
        return 26 + (c - '0');
    }
    return -1;
}

static char character_at(unsigned int index)
{
    return (index < 26U) ? (char)('A' + index) : (char)('0' + (index - 26U));
}

route_command_t barcode_map_get(char character)
{
    int index = index_of(character);
    return (index < 0) ? ROUTE_COMMAND_NONE : (route_command_t)map_table[index];
}

bool barcode_map_set(char character, route_command_t command)
{
    int index = index_of(character);

    if ((index < 0) || (command > ROUTE_COMMAND_UTURN))
    {
        return false;
    }
    map_table[index] = (uint8_t)command;
    return true;
}

void barcode_map_clear(void)
{
    unsigned int index;

    for (index = 0U; index < BARCODE_MAP_SIZE; index++)
    {
        map_table[index] = (uint8_t)ROUTE_COMMAND_NONE;
    }
}

void barcode_map_defaults(void)
{
    barcode_map_clear();
    (void)barcode_map_set('A', ROUTE_COMMAND_LEFT);
    (void)barcode_map_set('B', ROUTE_COMMAND_RIGHT);
    (void)barcode_map_set('C', ROUTE_COMMAND_STRAIGHT);
    (void)barcode_map_set('D', ROUTE_COMMAND_UTURN);
}

char barcode_map_command_letter(route_command_t command)
{
    switch (command)
    {
        case ROUTE_COMMAND_LEFT: return 'L';
        case ROUTE_COMMAND_RIGHT: return 'R';
        case ROUTE_COMMAND_STRAIGHT: return 'S';
        case ROUTE_COMMAND_UTURN: return 'U';
        default: return '-';
    }
}

/* ---- Parsing ---------------------------------------------------------- */

typedef struct
{
    const char *start;
    unsigned int length;
} word_t;

static bool is_separator(char c)
{
    return (c == ' ') || (c == ',') || (c == '=') || (c == ':') ||
           (c == '\t') || (c == '\r') || (c == '\n') || (c == ';');
}

/* Split the text into words. Returns the count, or -1 if too many. */
static int split_words(const char *text, unsigned int length, word_t *words)
{
    unsigned int i = 0U;
    int count = 0;

    while (i < length)
    {
        while ((i < length) && is_separator(text[i]))
        {
            i++;
        }
        if (i >= length)
        {
            break;
        }
        if (count >= (int)MAP_MAX_WORDS)
        {
            return -1;
        }
        words[count].start = &text[i];
        words[count].length = 0U;
        while ((i < length) && !is_separator(text[i]))
        {
            words[count].length++;
            i++;
        }
        count++;
    }
    return count;
}

/* Case-insensitive compare of a word with an upper-case keyword. */
static bool word_is(const word_t *word, const char *keyword)
{
    unsigned int i;

    for (i = 0U; i < word->length; i++)
    {
        if ((keyword[i] == '\0') || (upper(word->start[i]) != keyword[i]))
        {
            return false;
        }
    }
    return keyword[i] == '\0';
}

static bool parse_command(const word_t *word, route_command_t *command)
{
    if (word_is(word, "LEFT") || word_is(word, "L"))
    {
        *command = ROUTE_COMMAND_LEFT;
    }
    else if (word_is(word, "RIGHT") || word_is(word, "R"))
    {
        *command = ROUTE_COMMAND_RIGHT;
    }
    else if (word_is(word, "STRAIGHT") || word_is(word, "S"))
    {
        *command = ROUTE_COMMAND_STRAIGHT;
    }
    else if (word_is(word, "UTURN") || word_is(word, "U") ||
             word_is(word, "U-TURN"))
    {
        *command = ROUTE_COMMAND_UTURN;
    }
    else if (word_is(word, "NONE") || word_is(word, "X"))
    {
        *command = ROUTE_COMMAND_NONE;
    }
    else
    {
        return false;
    }
    return true;
}

/* Bounded string append; always leaves the buffer NUL-terminated. */
static void append(char *reply, unsigned int size, unsigned int *used, const char *text)
{
    while ((*text != '\0') && ((*used + 1U) < size))
    {
        reply[(*used)++] = *text++;
    }
    reply[*used] = '\0';
}

static void append_word(char *reply, unsigned int size, unsigned int *used,
                        const word_t *word)
{
    char copy[MAP_MAX_WORD + 1U];
    unsigned int i;
    unsigned int n = (word->length < MAP_MAX_WORD) ? word->length : MAP_MAX_WORD;

    for (i = 0U; i < n; i++)
    {
        copy[i] = word->start[i];
    }
    copy[n] = '\0';
    append(reply, size, used, copy);
}

static void reply_map(char *reply, unsigned int size)
{
    unsigned int used = 0U;
    unsigned int index;
    bool any = false;
    char entry[5];

    append(reply, size, &used, "OK");
    for (index = 0U; index < BARCODE_MAP_SIZE; index++)
    {
        route_command_t command = (route_command_t)map_table[index];
        if (command != ROUTE_COMMAND_NONE)
        {
            entry[0] = ' ';
            entry[1] = character_at(index);
            entry[2] = '=';
            entry[3] = barcode_map_command_letter(command);
            entry[4] = '\0';
            append(reply, size, &used, entry);
            any = true;
        }
    }
    if (!any)
    {
        append(reply, size, &used, " (empty)");
    }
}

static bool reply_error(char *reply, unsigned int size, const char *reason,
                        const word_t *word)
{
    unsigned int used = 0U;

    if ((reply != NULL) && (size > 0U))
    {
        reply[0] = '\0';
        append(reply, size, &used, "ERR ");
        append(reply, size, &used, reason);
        if (word != NULL)
        {
            append(reply, size, &used, ": ");
            append_word(reply, size, &used, word);
        }
    }
    return false;
}

bool barcode_map_is_command(const char *text, unsigned int length)
{
    word_t words[1];
    unsigned int i = 0U;

    if (text == NULL)
    {
        return false;
    }
    while ((i < length) && is_separator(text[i]))
    {
        i++;
    }
    words[0].start = &text[i];
    words[0].length = 0U;
    while ((i < length) && !is_separator(text[i]))
    {
        words[0].length++;
        i++;
    }
    return word_is(&words[0], "MAP");
}

bool barcode_map_command(
    const char *text,
    unsigned int length,
    char *reply,
    unsigned int reply_size)
{
    word_t words[MAP_MAX_WORDS];
    char pending_char[BARCODE_MAP_SIZE];
    route_command_t pending_cmd[BARCODE_MAP_SIZE];
    unsigned int pairs = 0U;
    int count;
    int next = 1;
    bool clear = false;
    bool defaults = false;
    unsigned int i;

    if ((text == NULL) || !barcode_map_is_command(text, length))
    {
        return reply_error(reply, reply_size, "not a MAP command", NULL);
    }
    count = split_words(text, length, words);
    if (count < 0)
    {
        return reply_error(reply, reply_size, "too many words", NULL);
    }
    if (count == 1)
    {
        return reply_error(reply, reply_size,
                           "usage MAP <letter> <LEFT|RIGHT|STRAIGHT|UTURN> ...", NULL);
    }

    /* Optional first keyword. */
    if (word_is(&words[1], "SHOW"))
    {
        if (count != 2)
        {
            return reply_error(reply, reply_size, "SHOW takes nothing after it", NULL);
        }
        if ((reply != NULL) && (reply_size > 0U))
        {
            reply_map(reply, reply_size);
        }
        return true;
    }
    if (word_is(&words[1], "CLEAR"))
    {
        clear = true;
        next = 2;
    }
    else if (word_is(&words[1], "DEFAULT"))
    {
        defaults = true;
        next = 2;
    }

    /* Check every pair BEFORE changing anything (all-or-nothing). */
    if (((count - next) % 2) != 0)
    {
        return reply_error(reply, reply_size, "letter without a command",
                           &words[count - 1]);
    }
    for (; next < count; next += 2)
    {
        route_command_t command;

        if ((words[next].length != 1U) || (index_of(words[next].start[0]) < 0))
        {
            return reply_error(reply, reply_size, "bad letter", &words[next]);
        }
        if (!parse_command(&words[next + 1], &command))
        {
            return reply_error(reply, reply_size, "bad command", &words[next + 1]);
        }
        if (pairs >= BARCODE_MAP_SIZE)
        {
            return reply_error(reply, reply_size, "too many pairs", NULL);
        }
        pending_char[pairs] = words[next].start[0];
        pending_cmd[pairs] = command;
        pairs++;
    }

    /* All valid: apply. */
    if (clear)
    {
        barcode_map_clear();
    }
    if (defaults)
    {
        barcode_map_defaults();
    }
    for (i = 0U; i < pairs; i++)
    {
        (void)barcode_map_set(pending_char[i], pending_cmd[i]);
    }
    if ((reply != NULL) && (reply_size > 0U))
    {
        reply_map(reply, reply_size);
    }
    return true;
}
