//
// Memory-direct WAD I/O for the OoO RISC-V port.  (Replaces the upstream w_file_stdc.c.)
//
// The upstream stdc backend reads the WAD through FILE* / fread / fseek. On this bare-metal
// picolibc target that descends into byte-at-a-time buffered stdio (getc, __bufio_get,
// __bufio_seek), so merely loading the 21 MB WAD costs on the order of 600M instructions -- which
// is exactly what kept Doom from ever reaching a rendered frame in a reasonable simulation.
//
// The WAD is already resident in memory: wad.S embeds it as the `doom1_wad` array. So this backend
//   - sets wad->mapped = doom1_wad, which makes W_CacheLumpNum hand back pointers DIRECTLY into the
//     WAD (zero-copy lump access -- no read, no memcpy, no stdio), and
//   - implements Read as a plain memcpy from the array (used only for the small header/directory
//     reads at startup).
// No FILE*, no stdio, no per-byte getc.
//

#include <stddef.h>
#include <string.h>

#include "w_file.h"
#include "z_zone.h"

extern const unsigned char doom1_wad[];
extern const unsigned int  doom1_wad_len;

extern wad_file_class_t stdc_wad_file;

static wad_file_t *W_StdC_OpenFile(char *path)
{
    (void) path; // the image contains exactly one WAD, resident in memory

    wad_file_t *wad = Z_Malloc(sizeof(wad_file_t), PU_STATIC, 0);
    wad->file_class = &stdc_wad_file;
    wad->mapped     = (byte *) doom1_wad; // enables zero-copy lump caching in W_CacheLumpNum
    wad->length     = doom1_wad_len;
    return wad;
}

static void W_StdC_CloseFile(wad_file_t *wad)
{
    Z_Free(wad);
}

size_t W_StdC_Read(wad_file_t *wad, unsigned int offset,
                   void *buffer, size_t buffer_len)
{
    (void) wad;

    if (offset >= doom1_wad_len)
    {
        return 0;
    }

    size_t avail = (size_t) (doom1_wad_len - offset);
    if (buffer_len > avail)
    {
        buffer_len = avail;
    }

    memcpy(buffer, doom1_wad + offset, buffer_len);
    return buffer_len;
}

wad_file_class_t stdc_wad_file =
{
    W_StdC_OpenFile,
    W_StdC_CloseFile,
    W_StdC_Read,
};
