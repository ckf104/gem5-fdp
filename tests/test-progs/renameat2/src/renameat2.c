#include <errno.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef RENAME_NOREPLACE
#define RENAME_NOREPLACE (1U << 0)
#endif

#ifndef RENAME_EXCHANGE
#define RENAME_EXCHANGE (1U << 1)
#endif

#if defined(SYS_renameat2)
#define GEM5_RENAMEAT2_NR SYS_renameat2
#elif defined(__NR_renameat2)
#define GEM5_RENAMEAT2_NR __NR_renameat2
#else
#error "renameat2 syscall number is not available on this toolchain"
#endif

static int
call_renameat2(const char *old_path, const char *new_path, unsigned int flags)
{
    return syscall(
        GEM5_RENAMEAT2_NR, AT_FDCWD, old_path, AT_FDCWD, new_path, flags);
}

static bool
write_text_file(const char *path, const char *text)
{
    FILE *file = fopen(path, "w");
    if (!file)
        return false;

    bool ok = fputs(text, file) >= 0;
    ok = ok && (fclose(file) == 0);
    return ok;
}

static bool
read_text_file(const char *path, char *buffer, size_t buffer_size)
{
    FILE *file = fopen(path, "r");
    if (!file)
        return false;

    if (!fgets(buffer, buffer_size, file)) {
        fclose(file);
        return false;
    }

    return fclose(file) == 0;
}

static bool
path_exists(const char *path)
{
    return access(path, F_OK) == 0;
}

int
main(void)
{
    bool all_pass = true;
    char template_dir[] = "/tmp/gem5-renameat2-XXXXXX";
    char *tmpdir = mkdtemp(template_dir);

    if (!tmpdir) {
        perror("mkdtemp");
        return 1;
    }

    char a[256], b[256], c[256], d[256], e[256], f[256];
    snprintf(a, sizeof(a), "%s/a.txt", tmpdir);
    snprintf(b, sizeof(b), "%s/b.txt", tmpdir);
    snprintf(c, sizeof(c), "%s/c.txt", tmpdir);
    snprintf(d, sizeof(d), "%s/d.txt", tmpdir);
    snprintf(e, sizeof(e), "%s/e.txt", tmpdir);
    snprintf(f, sizeof(f), "%s/f.txt", tmpdir);

    if (!write_text_file(a, "A") || !write_text_file(c, "SRC") ||
        !write_text_file(d, "DST") || !write_text_file(e, "LEFT") ||
        !write_text_file(f, "RIGHT")) {
        perror("write_text_file");
        return 1;
    }

    bool pass_flags_0 = false;
    if (call_renameat2(a, b, 0) == 0) {
        char content[32] = {0};
        pass_flags_0 = !path_exists(a) && path_exists(b) &&
                       read_text_file(b, content, sizeof(content)) &&
                       strcmp(content, "A") == 0;
    }

    printf("flags=0: %s\n", pass_flags_0 ? "PASS" : "FAIL");
    all_pass = all_pass && pass_flags_0;

    errno = 0;
    bool pass_noreplace = false;
    if (call_renameat2(c, d, RENAME_NOREPLACE) == -1 && errno == EEXIST) {
        char src_content[32] = {0};
        char dst_content[32] = {0};
        pass_noreplace = path_exists(c) && path_exists(d) &&
                         read_text_file(c, src_content, sizeof(src_content)) &&
                         read_text_file(d, dst_content, sizeof(dst_content)) &&
                         strcmp(src_content, "SRC") == 0 &&
                         strcmp(dst_content, "DST") == 0;
    }

    printf("RENAME_NOREPLACE: %s\n", pass_noreplace ? "PASS" : "FAIL");
    all_pass = all_pass && pass_noreplace;

    bool pass_exchange = false;
    if (call_renameat2(e, f, RENAME_EXCHANGE) == 0) {
        char e_content[32] = {0};
        char f_content[32] = {0};
        pass_exchange = read_text_file(e, e_content, sizeof(e_content)) &&
                        read_text_file(f, f_content, sizeof(f_content)) &&
                        strcmp(e_content, "RIGHT") == 0 &&
                        strcmp(f_content, "LEFT") == 0;
    }

    printf("RENAME_EXCHANGE: %s\n", pass_exchange ? "PASS" : "FAIL");
    all_pass = all_pass && pass_exchange;

    unlink(a);
    unlink(b);
    unlink(c);
    unlink(d);
    unlink(e);
    unlink(f);
    rmdir(tmpdir);

    printf("renameat2-test: %s\n", all_pass ? "PASS" : "FAIL");
    return all_pass ? 0 : 1;
}
