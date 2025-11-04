default:
    @just --list

build arch="arm64":
    #!/usr/bin/env bash
    set -e
    export DEB_BUILD_OPTIONS="parallel=$(nproc)"
    debuild -us -uc -b -a{{ arch }} -d --lintian-opts --profile=debian
    mkdir -p dist && mv ../*.deb dist/ 2>/dev/null || true
    rm -f ../*.{dsc,tar.*,changes,buildinfo,build}

changelog:
    gbp dch -R --ignore-branch --release

clean:
    make clean 2>/dev/null || true
    rm -rf debian/.debhelper debian/files debian/*.log debian/*.substvars debian/pamir-ai-sam-dkms debian/debhelper-build-stamp dist
    rm -f ../*.deb ../*.dsc ../*.tar.* ../*.changes ../*.buildinfo ../*.build
    rm -f *.ko *.o *.mod.c *.mod.o .*.cmd Module.symvers modules.order *.dtbo
    rm -rf .tmp_versions

# DKMS project recipes
module:
    make
