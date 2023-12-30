/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  implement a file store for embedded firmware images
 */

#include "AP_ROMFS.h"
#include "tinf.h"
#include <AP_Math/crc.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include <string.h>

#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_H
#include <ap_romfs_embedded.h>
#else
const AP_ROMFS::embedded_file AP_ROMFS::files[] = {};
#endif


static size_t
armthumb_code(void *simple,
        uint32_t now_pos, bool is_encoder,
        uint8_t *buffer, size_t size)
{
    size_t i;
    for (i = 0; i + 4 <= size; i += 2) {
        if ((buffer[i + 1] & 0xF8) == 0xF0
                && (buffer[i + 3] & 0xF8) == 0xF8) {
            uint32_t src = (((uint32_t)(buffer[i + 1]) & 7) << 19)
                | ((uint32_t)(buffer[i + 0]) << 11)
                | (((uint32_t)(buffer[i + 3]) & 7) << 8)
                | (uint32_t)(buffer[i + 2]);

            src <<= 1;

            uint32_t dest;
            if (is_encoder)
                dest = now_pos + (uint32_t)(i) + 4 + src;
            else
                dest = src - (now_pos + (uint32_t)(i) + 4);

            dest >>= 1;
            buffer[i + 1] = 0xF0 | ((dest >> 19) & 0x7);
            buffer[i + 0] = (dest >> 11);
            buffer[i + 3] = 0xF8 | ((dest >> 8) & 0x7);
            buffer[i + 2] = (dest);
            i += 2;
        }
    }

    return i;
}

/*
  find an embedded file
*/
const uint8_t *AP_ROMFS::find_file(const char *name, uint32_t &size, uint32_t &crc)
{
    for (uint16_t i=0; i<ARRAY_SIZE(files); i++) {
        if (strcmp(name, files[i].filename) == 0) {
            size = files[i].size;
            crc = files[i].crc;
            return files[i].contents;
        }
    }
    return nullptr;
}

/*
  find a compressed file and uncompress it. Space for decompressed
  data comes from malloc. Caller must be careful to free the resulting
  data after use. The next byte after the file data is guaranteed to
  be null
*/
const uint8_t *AP_ROMFS::find_decompress(const char *name, uint32_t &size)
{
    uint32_t compressed_size = 0;
    uint32_t crc;
    const uint8_t *compressed_data = find_file(name, compressed_size, crc);
    if (!compressed_data) {
        return nullptr;
    }

#ifdef HAL_ROMFS_UNCOMPRESSED
    size = compressed_size;
    return compressed_data;
#else
    // last 4 bytes of gzip file are length of decompressed data
    const uint8_t *p = &compressed_data[compressed_size-4];
    uint32_t decompressed_size = p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
    
    uint8_t *decompressed_data = (uint8_t *)malloc(decompressed_size + 1);
    if (!decompressed_data) {
        return nullptr;
    }

    // explicitly null terimnate the data
    decompressed_data[decompressed_size] = 0;

    TINF_DATA *d = (TINF_DATA *)malloc(sizeof(TINF_DATA));
    if (!d) {
        ::free(decompressed_data);
        return nullptr;
    }
    uzlib_uncompress_init(d, NULL, 0);

    d->source = compressed_data;
    d->source_limit = compressed_data + compressed_size - 4;

    d->dest = decompressed_data;
    d->destSize = decompressed_size;

    // we don't check CRC, as it just wastes flash space for constant
    // ROMFS data
    int res = uzlib_uncompress(d);

    ::free(d);
    
    if (res != TINF_OK) {
        ::free(decompressed_data);
        return nullptr;
    }

    armthumb_code(NULL, 0, false, decompressed_data, decompressed_size);

    if (crc32_small(0, decompressed_data, decompressed_size) != crc) {
        ::free(decompressed_data);
        return nullptr;
    }
    
    size = decompressed_size;
    return decompressed_data;
#endif
}

// free returned data
void AP_ROMFS::free(const uint8_t *data)
{
#ifndef HAL_ROMFS_UNCOMPRESSED
    ::free(const_cast<uint8_t *>(data));
#endif
}

/*
  directory listing interface. Start with ofs=0. Returns pathnames
  that match dirname prefix. Ends with nullptr return when no more
  files found
*/
const char *AP_ROMFS::dir_list(const char *dirname, uint16_t &ofs)
{
    const size_t dlen = strlen(dirname);
    for ( ; ofs < ARRAY_SIZE(files); ofs++) {
        if (strncmp(dirname, files[ofs].filename, dlen) == 0 &&
            files[ofs].filename[dlen] == '/') {
            // found one
            return files[ofs++].filename;
        }
    }
    return nullptr;
}
