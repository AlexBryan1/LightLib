/*
 * font.c
 * Hand-crafted 8×14 bold angular bitmap font — aesthetic.
 * Characters: A B C F G I K M N O  (all that are needed for BACK / INFO / IMG)
 *
 * Glyph design: 8 pixels wide, variable height, 1 bpp row-major MSB-first.
 * adv_w = 11 px × 16 = 176 (in LVGL 8.4 fixed-point units).
 * line_height = 16, base_line = 2.
 * ofs_y is measured from the TOP of the line (positive = further down).
 */

#include "liblvgl/lvgl.h"
#include "liblvgl/font/lv_font_fmt_txt.h"

/* ── Bitmap data ─────────────────────────────────────────────────────────── */
/*
 * Each byte = one row of 8 pixels, MSB is leftmost.
 * Characters are grouped in codepoint order: A B C F G I K M N O
 *
 * Visual key (8 pixels wide):
 *   76543210  ← bit position
 *   ##......  = 0xC0
 *   ########  = 0xFF
 */

static const uint8_t glyph_bitmap[] = {

    /* A  — bold wedge with double crossbar */
    0x18,  /* ...##...  */
    0x3C,  /* ..####..  */
    0x7E,  /* .######.  */
    0xE7,  /* ###..###  */
    0xC3,  /* ##....##  */
    0xFF,  /* ########  */
    0xFF,  /* ########  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    /* 10 bytes, offset 0 */

    /* B  — bold B with angular joins */
    0xFC,  /* ######..  */
    0xFE,  /* #######.  */
    0xC7,  /* ##...###  */
    0xC3,  /* ##....##  */
    0xFE,  /* #######.  */
    0xFF,  /* ########  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC7,  /* ##...###  */
    0xFE,  /* #######.  */
    0xFC,  /* ######..  */
    /* 11 bytes, offset 10 */

    /* C  — bold C with double top/bottom serif */
    0x3C,  /* ..####..  */
    0x7E,  /* .######.  */
    0xFF,  /* ########  */
    0xC0,  /* ##......  */
    0xC0,  /* ##......  */
    0xC0,  /* ##......  */
    0xC0,  /* ##......  */
    0xFF,  /* ########  */
    0x7E,  /* .######.  */
    0x3C,  /* ..####..  */
    /* 10 bytes, offset 21 */

    /* F  — bold F with double top bar */
    0xFF,  /* ########  */
    0xFF,  /* ########  */
    0xC0,  /* ##......  */
    0xC0,  /* ##......  */
    0xFC,  /* ######..  */
    0xFC,  /* ######..  */
    0xC0,  /* ##......  */
    0xC0,  /* ##......  */
    0xC0,  /* ##......  */
    /* 9 bytes, offset 31 */

    /* G  — bold G with inner shelf */
    0x3C,  /* ..####..  */
    0x7E,  /* .######.  */
    0xFF,  /* ########  */
    0xC0,  /* ##......  */
    0xCF,  /* ##..####  */
    0xCF,  /* ##..####  */
    0xC3,  /* ##....##  */
    0xFF,  /* ########  */
    0x7E,  /* .######.  */
    0x3C,  /* ..####..  */
    /* 10 bytes, offset 40 */

    /* I  — bold I with double top & bottom serif */
    0xFF,  /* ########  */
    0xFF,  /* ########  */
    0x18,  /* ...##...  */
    0x18,  /* ...##...  */
    0x18,  /* ...##...  */
    0x18,  /* ...##...  */
    0x18,  /* ...##...  */
    0x18,  /* ...##...  */
    0x18,  /* ...##...  */
    0x18,  /* ...##...  */
    0xFF,  /* ########  */
    0xFF,  /* ########  */
    /* 12 bytes, offset 50 */

    /* K  — bold K with angular diagonal */
    0xC6,  /* ##...##.  */
    0xCC,  /* ##..##..  */
    0xD8,  /* ##.##...  */
    0xF0,  /* ####....  */
    0xE0,  /* ###.....  */
    0xF0,  /* ####....  */
    0xD8,  /* ##.##...  */
    0xCC,  /* ##..##..  */
    0xC6,  /* ##...##.  */
    0xC3,  /* ##....##  */
    /* 10 bytes, offset 62 */

    /* M  — bold M with double peak */
    0xC3,  /* ##....##  */
    0xE7,  /* ###..###  */
    0xFF,  /* ########  */
    0xFF,  /* ########  */
    0xDB,  /* ##.##.##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    /* 10 bytes, offset 72 */

    /* N  — bold N with diagonal stroke */
    0xC3,  /* ##....##  */
    0xE3,  /* ###...##  */
    0xF3,  /* ####..##  */
    0xDB,  /* ##.##.##  */
    0xCB,  /* ##..#.##  */
    0xC7,  /* ##...###  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    /* 10 bytes, offset 82 */

    /* O  — bold O with thick frame */
    0x3C,  /* ..####..  */
    0x7E,  /* .######.  */
    0xFF,  /* ########  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xC3,  /* ##....##  */
    0xFF,  /* ########  */
    0x7E,  /* .######.  */
    0x3C,  /* ..####..  */
    /* 10 bytes, offset 92 */
};

/* ── Glyph descriptors ────────────────────────────────────────────────────── */
/*
 * adv_w = 11 * 16 = 176  (11 px advance)
 * ofs_y: measured from top of line (positive = further from top edge)
 *   line_height=16, base_line=2
 *   For box_h=10: ofs_y=2 → glyph drawn from y+2 to y+12
 *   For box_h=12: ofs_y=1 → glyph drawn from y+1 to y+13
 */
static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0,  .adv_w = 0,   .box_w = 0, .box_h = 0,  .ofs_x = 0, .ofs_y = 0}, /* [0] placeholder */
    {.bitmap_index = 0,  .adv_w = 176, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 2}, /* [1] A  */
    {.bitmap_index = 10, .adv_w = 176, .box_w = 8, .box_h = 11, .ofs_x = 0, .ofs_y = 1}, /* [2] B  */
    {.bitmap_index = 21, .adv_w = 176, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 2}, /* [3] C  */
    {.bitmap_index = 31, .adv_w = 176, .box_w = 8, .box_h = 9,  .ofs_x = 0, .ofs_y = 2}, /* [4] F  */
    {.bitmap_index = 40, .adv_w = 176, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 2}, /* [5] G  */
    {.bitmap_index = 50, .adv_w = 176, .box_w = 8, .box_h = 12, .ofs_x = 0, .ofs_y = 1}, /* [6] I  */
    {.bitmap_index = 62, .adv_w = 176, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 2}, /* [7] K  */
    {.bitmap_index = 72, .adv_w = 176, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 2}, /* [8] M  */
    {.bitmap_index = 82, .adv_w = 176, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 2}, /* [9] N  */
    {.bitmap_index = 92, .adv_w = 176, .box_w = 8, .box_h = 10, .ofs_x = 0, .ofs_y = 2}, /* [10] O */
};

/* ── Character map ────────────────────────────────────────────────────────── */
/*
 * SPARSE_TINY: unicode_list holds offsets from range_start (65 = 'A').
 *   A=0, B=1, C=2, F=5, G=6, I=8, K=10, M=12, N=13, O=14
 */
static const uint16_t unicode_list[] = {0, 1, 2, 5, 6, 8, 10, 12, 13, 14};

static const lv_font_fmt_txt_cmap_t cmaps[] = {
    {
        .range_start      = 65,
        .range_length     = 15,          /* covers 'A' (65) through 'O' (79) */
        .glyph_id_start   = 1,
        .unicode_list     = unicode_list,
        .glyph_id_ofs_list= NULL,
        .list_length      = 10,
        .type             = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY,
    }
};

/* ── Font descriptor ──────────────────────────────────────────────────────── */
static lv_font_fmt_txt_dsc_t font_dsc = {
    .glyph_bitmap  = glyph_bitmap,
    .glyph_dsc     = glyph_dsc,
    .cmaps         = cmaps,
    .kern_dsc      = NULL,
    .kern_scale    = 0,
    .cmap_num      = 1,
    .bpp           = 1,
    .kern_classes  = 0,
    .bitmap_format = 0,
    .cache         = NULL,
};

/* ── Public font object ───────────────────────────────────────────────────── */
lv_font_t FONT = {
    .get_glyph_dsc    = lv_font_get_glyph_dsc_fmt_txt,
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,
    .line_height      = 16,
    .base_line        = 2,
    .subpx            = LV_FONT_SUBPX_NONE,
    .underline_position  = -1,
    .underline_thickness =  1,
    .dsc              = &font_dsc,
};
