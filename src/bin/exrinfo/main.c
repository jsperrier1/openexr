/*
** SPDX-License-Identifier: BSD-3-Clause
** Copyright Contributors to the OpenEXR Project.
*/

#include <openexr.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#    include <fcntl.h>
#    include <io.h>
#    include <windows.h>
#else
#    include <unistd.h>
#endif

#include <math.h>
#include <stdlib.h>

static void
usage (FILE* stream, const char* argv0, int verbose)
{
    fprintf (
        stream,
        "Usage: %s [-v|--verbose] [-a|--all-metadata] [-s|--strict] [-m|--minmax] <filename> [<filename> ...]\n\n",
        argv0);

    if (verbose)
        fprintf (
            stream,
            "\n"
            "Read exr files and print values of header attributes\n"
            "\n"
            "Options:\n"
            "  -s, --strict        strict mode\n"
            "  -a, --all-metadata  print all metadata\n"
            "  -m, --minmax        print min/max pixel values per channel\n"
            "  -v, --verbose       verbose mode\n"
            "  -h, --help          print this message\n"
            "      --version       print version information\n"
            "\n"
            "Report bugs via https://github.com/AcademySoftwareFoundation/openexr/issues or email security@openexr.com\n"
            "");
}

static void
error_handler_cb (exr_const_context_t f, int code, const char* msg)
{
    const char* fn;
    if (EXR_ERR_SUCCESS != exr_get_file_name (f, &fn)) fn = "<error>";
    fprintf (
        stderr,
        "ERROR '%s' (%s): %s\n",
        fn,
        exr_get_error_code_as_string (code),
        msg);
}

static int64_t
stdin_reader (
    exr_const_context_t         file,
    void*                       userdata,
    void*                       buffer,
    uint64_t                    sz,
    uint64_t                    offset,
    exr_stream_error_func_ptr_t error_cb)
{
    static uint64_t lastoffset = 0;
    int64_t         nread      = 0;

    (void) userdata;

    if (offset != lastoffset)
    {
        error_cb (file, EXR_ERR_READ_IO, "Unable to seek in stdin stream");
        return -1;
    }
#ifdef _WIN32
    if (sz >= (size_t) (INT32_MAX))
    {
        error_cb (
            file, EXR_ERR_READ_IO, "Read request too large for win32 API");
        return -1;
    }
    nread = _read (_fileno (stdin), buffer, (unsigned) sz);
#else
    nread = read (STDIN_FILENO, buffer, sz);
#endif
    if (nread > 0) lastoffset = offset + (uint64_t) nread;
    return nread;
}

/* ------------------------------------------------------------------ */
/* Half-float to float conversion                                       */
/* ------------------------------------------------------------------ */

static float
half_to_float_val (uint16_t h)
{
    uint32_t s = (uint32_t) (h >> 15) << 31;
    uint32_t e = (h >> 10) & 0x1fu;
    uint32_t m = h & 0x3ffu;
    uint32_t r;

    if (e == 0)
    {
        if (m == 0)
        {
            r = s;
        }
        else
        {
            /* denormalized: normalize it */
            while (!(m & 0x400u)) { m <<= 1; --e; }
            ++e;
            m &= 0x3ffu;
            r = s | ((e + 112u) << 23) | (m << 13);
        }
    }
    else if (e == 31)
    {
        r = s | 0x7f800000u | (m << 13); /* Inf or NaN */
    }
    else
    {
        r = s | ((e + 112u) << 23) | (m << 13);
    }

    float f;
    memcpy (&f, &r, sizeof (f));
    return f;
}

typedef struct
{
    double min_val;
    double max_val;
    int    initialized;
} chan_stats_t;

static void
update_stats (
    chan_stats_t*  st,
    const uint8_t* data,
    int32_t        npix,
    int32_t        pixel_stride,
    uint16_t       dtype)
{
    for (int32_t i = 0; i < npix; ++i, data += pixel_stride)
    {
        double v;
        if (dtype == EXR_PIXEL_HALF)
        {
            uint16_t hv;
            memcpy (&hv, data, 2);
            float fv = half_to_float_val (hv);
            if (isnan (fv)) continue;
            v = (double) fv;
        }
        else if (dtype == EXR_PIXEL_FLOAT)
        {
            float fv;
            memcpy (&fv, data, 4);
            if (isnan (fv)) continue;
            v = (double) fv;
        }
        else /* EXR_PIXEL_UINT */
        {
            uint32_t uv;
            memcpy (&uv, data, 4);
            v = (double) uv;
        }

        if (!st->initialized || v < st->min_val) st->min_val = v;
        if (!st->initialized || v > st->max_val) st->max_val = v;
        st->initialized = 1;
    }
}

static void
print_stats (exr_const_context_t e)
{
    int          part_count = 0;
    exr_result_t rv;

    rv = exr_get_count (e, &part_count);
    if (rv != EXR_ERR_SUCCESS) return;

    for (int part = 0; part < part_count; part++)
    {
        exr_storage_t storage;
        rv = exr_get_storage (e, part, &storage);
        if (rv != EXR_ERR_SUCCESS) continue;

        if (storage == EXR_STORAGE_DEEP_SCANLINE ||
            storage == EXR_STORAGE_DEEP_TILED)
        {
            if (part_count > 1)
                printf (
                    "part %d: pixel stats not supported for deep data\n", part);
            else
                printf ("pixel stats: not supported for deep data\n");
            continue;
        }

        const exr_attr_chlist_t* chlist = NULL;
        rv = exr_get_channels (e, part, &chlist);
        if (rv != EXR_ERR_SUCCESS || !chlist || chlist->num_channels == 0)
            continue;

        int           nchan = chlist->num_channels;
        chan_stats_t* stats = calloc ((size_t) nchan, sizeof (chan_stats_t));
        if (!stats) continue;

        uint64_t max_unpacked = 0;
        exr_get_chunk_unpacked_size (e, part, &max_unpacked);
        if (max_unpacked == 0) max_unpacked = 65536;

        uint8_t* chan_buf = malloc (max_unpacked);
        if (!chan_buf)
        {
            free (stats);
            continue;
        }

        exr_decode_pipeline_t decode = EXR_DECODE_PIPELINE_INITIALIZER;

        if (storage == EXR_STORAGE_SCANLINE)
        {
            exr_attr_box2i_t dw  = {{0, 0}, {0, 0}};
            int32_t          spc = 1;
            exr_get_data_window (e, part, &dw);
            exr_get_scanlines_per_chunk (e, part, &spc);

            for (int y = dw.min.y; y <= dw.max.y; y += spc)
            {
                exr_chunk_info_t cinfo = {0};
                rv = exr_read_scanline_chunk_info (e, part, y, &cinfo);
                if (rv != EXR_ERR_SUCCESS) continue;

                if (decode.channels == NULL)
                {
                    rv = exr_decoding_initialize (e, part, &cinfo, &decode);
                    if (rv != EXR_ERR_SUCCESS) break;

                    uint8_t* ptr = chan_buf;
                    for (int c = 0; c < decode.channel_count; c++)
                    {
                        exr_coding_channel_info_t* ch = &decode.channels[c];
                        ch->decode_to_ptr          = ptr;
                        ch->user_bytes_per_element = ch->bytes_per_element;
                        ch->user_data_type         = ch->data_type;
                        ch->user_pixel_stride      = ch->bytes_per_element;
                        ch->user_line_stride =
                            ch->width * (int32_t) ch->bytes_per_element;
                        ptr += (size_t) ch->width * (size_t) ch->height *
                               (size_t) ch->bytes_per_element;
                    }

                    rv = exr_decoding_choose_default_routines (
                        e, part, &decode);
                    if (rv != EXR_ERR_SUCCESS) break;
                }
                else
                {
                    rv = exr_decoding_update (e, part, &cinfo, &decode);
                    if (rv != EXR_ERR_SUCCESS) continue;

                    uint8_t* ptr = chan_buf;
                    for (int c = 0; c < decode.channel_count; c++)
                    {
                        exr_coding_channel_info_t* ch = &decode.channels[c];
                        ch->decode_to_ptr = ptr;
                        ptr += (size_t) ch->width * (size_t) ch->height *
                               (size_t) ch->bytes_per_element;
                    }
                }

                rv = exr_decoding_run (e, part, &decode);
                if (rv != EXR_ERR_SUCCESS) continue;

                uint8_t* ptr = chan_buf;
                for (int c = 0; c < decode.channel_count; c++)
                {
                    exr_coding_channel_info_t* ch = &decode.channels[c];
                    update_stats (
                        &stats[c],
                        ptr,
                        ch->width * ch->height,
                        ch->bytes_per_element,
                        ch->data_type);
                    ptr += (size_t) ch->width * (size_t) ch->height *
                           (size_t) ch->bytes_per_element;
                }
            }
        }
        else /* EXR_STORAGE_TILED */
        {
            int32_t countx = 0, county = 0;
            rv = exr_get_tile_counts (e, part, 0, 0, &countx, &county);
            if (rv == EXR_ERR_SUCCESS)
            {
                int failed = 0;
                for (int ty = 0; ty < county && !failed; ty++)
                {
                    for (int tx = 0; tx < countx && !failed; tx++)
                    {
                        exr_chunk_info_t cinfo = {0};
                        rv = exr_read_tile_chunk_info (
                            e, part, tx, ty, 0, 0, &cinfo);
                        if (rv != EXR_ERR_SUCCESS) continue;

                        if (decode.channels == NULL)
                        {
                            rv = exr_decoding_initialize (
                                e, part, &cinfo, &decode);
                            if (rv != EXR_ERR_SUCCESS)
                            {
                                failed = 1;
                                break;
                            }

                            uint8_t* ptr = chan_buf;
                            for (int c = 0; c < decode.channel_count; c++)
                            {
                                exr_coding_channel_info_t* ch =
                                    &decode.channels[c];
                                ch->decode_to_ptr = ptr;
                                ch->user_bytes_per_element =
                                    ch->bytes_per_element;
                                ch->user_data_type    = ch->data_type;
                                ch->user_pixel_stride = ch->bytes_per_element;
                                ch->user_line_stride =
                                    ch->width * (int32_t) ch->bytes_per_element;
                                ptr += (size_t) ch->width * (size_t) ch->height *
                                       (size_t) ch->bytes_per_element;
                            }

                            rv = exr_decoding_choose_default_routines (
                                e, part, &decode);
                            if (rv != EXR_ERR_SUCCESS)
                            {
                                failed = 1;
                                break;
                            }
                        }
                        else
                        {
                            rv = exr_decoding_update (e, part, &cinfo, &decode);
                            if (rv != EXR_ERR_SUCCESS) continue;

                            uint8_t* ptr = chan_buf;
                            for (int c = 0; c < decode.channel_count; c++)
                            {
                                exr_coding_channel_info_t* ch =
                                    &decode.channels[c];
                                ch->decode_to_ptr = ptr;
                                ptr += (size_t) ch->width *
                                       (size_t) ch->height *
                                       (size_t) ch->bytes_per_element;
                            }
                        }

                        rv = exr_decoding_run (e, part, &decode);
                        if (rv != EXR_ERR_SUCCESS) continue;

                        uint8_t* ptr = chan_buf;
                        for (int c = 0; c < decode.channel_count; c++)
                        {
                            exr_coding_channel_info_t* ch = &decode.channels[c];
                            update_stats (
                                &stats[c],
                                ptr,
                                ch->width * ch->height,
                                ch->bytes_per_element,
                                ch->data_type);
                            ptr += (size_t) ch->width * (size_t) ch->height *
                                   (size_t) ch->bytes_per_element;
                        }
                    }
                }
            }
        }

        if (decode.channels != NULL) exr_decoding_destroy (e, &decode);
        free (chan_buf);

        /* print results */
        if (part_count > 1)
        {
            const char* pname = NULL;
            exr_get_name (e, part, &pname);
            if (pname)
                printf ("part %d (%s) pixel stats:\n", part, pname);
            else
                printf ("part %d pixel stats:\n", part);
        }
        else
        {
            printf ("pixel stats:\n");
        }

        for (int c = 0; c < nchan; c++)
        {
            if (stats[c].initialized)
                printf (
                    "  channel %s: min = %g, max = %g\n",
                    chlist->entries[c].name.str,
                    stats[c].min_val,
                    stats[c].max_val);
            else
                printf (
                    "  channel %s: no data\n", chlist->entries[c].name.str);
        }

        free (stats);
    }
}

static int
process_stdin (int verbose, int allmeta, int strict, int minmax)
{
    int                       failcount = 0;
    exr_result_t              rv;
    exr_context_t             e     = NULL;
    exr_context_initializer_t cinit = EXR_DEFAULT_CONTEXT_INITIALIZER;
    cinit.error_handler_fn          = &error_handler_cb;
    cinit.read_fn                   = &stdin_reader;

    if (!verbose) cinit.flags |= EXR_CONTEXT_FLAG_SILENT_HEADER_PARSE;

    if (strict) cinit.flags |= EXR_CONTEXT_FLAG_STRICT_HEADER;

#ifdef _WIN32
    _setmode (_fileno (stdin), _O_BINARY);
#endif
    rv = exr_start_read (&e, "<stdin>", &cinit);
    if (rv == EXR_ERR_SUCCESS)
    {
        exr_print_context_info (e, verbose || allmeta);
        if (minmax) print_stats (e);
        exr_finish (&e);
    }
    else
        ++failcount;
    return failcount;
}

static int
process_file (const char* filename, int verbose, int allmeta, int strict, int minmax)
{
    int                       failcount = 0;
    exr_result_t              rv;
    exr_context_t             e     = NULL;
    exr_context_initializer_t cinit = EXR_DEFAULT_CONTEXT_INITIALIZER;
    cinit.error_handler_fn          = &error_handler_cb;

    if (!verbose) cinit.flags |= EXR_CONTEXT_FLAG_SILENT_HEADER_PARSE;

    if (strict) cinit.flags |= EXR_CONTEXT_FLAG_STRICT_HEADER;

    rv = exr_start_read (&e, filename, &cinit);

    if (rv == EXR_ERR_SUCCESS)
    {
        exr_print_context_info (e, verbose || allmeta);
        if (minmax) print_stats (e);
        exr_finish (&e);
    }
    else
        ++failcount;
    return failcount;
}

int
main (int argc, const char* argv[])
{
    int rv = 0, verbose = 0, allmeta = 0, strict = 0, minmax = 0;

    for (int a = 1; a < argc; ++a)
    {
        if (!strcmp (argv[a], "-h") || !strcmp (argv[a], "-?") ||
            !strcmp (argv[a], "--help"))
        {
            usage (stdout, "exrinfo", 1);
            return 0;
        }
        else if (!strcmp (argv[a], "--version"))
        {
            printf (
                "exrinfo (OpenEXR) %s  https://openexr.com\n",
                OPENEXR_VERSION_STRING);
            printf ("Copyright (c) Contributors to the OpenEXR Project\n");
            printf ("License BSD-3-Clause\n");
            return 0;
        }
        else if (!strcmp (argv[a], "-v") || !strcmp (argv[a], "--verbose"))
        {
            verbose = 1;
        }
        else if (!strcmp (argv[a], "-a") || !strcmp (argv[a], "--all-metadata"))
        {
            allmeta = 1;
        }
        else if (!strcmp (argv[a], "-s") || !strcmp (argv[a], "--strict"))
        {
            strict = 1;
        }
        else if (!strcmp (argv[a], "-m") || !strcmp (argv[a], "--minmax"))
        {
            minmax = 1;
        }
        else if (!strcmp (argv[a], "-"))
        {
            rv += process_stdin (verbose, allmeta, strict, minmax);
        }
        else if (argv[a][0] == '-')
        {
            usage (stderr, argv[0], 0);
            return 1;
        }
        else { rv += process_file (argv[a], verbose, allmeta, strict, minmax); }
    }

    return rv;
}
