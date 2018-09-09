SUMMARY = "Example of how to build an external Linux kernel module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

DEPENDS = "	\
	pasm-compiler \
	"


SRC_URI = "file://Makefile \
           file://hcsr04.c \
           file://hcsr04.h \
           file://hcsr04_FL.p \
           file://hcsr04_FC.p \
           file://hcsr04_FR.p \
           file://hcsr04_RL.p \
           file://hcsr04_RR.p \
           file://libprussdrv.a \
           file://prussdrv.h \
           file://pruss_intc_mapping.h \
          "

S = "${WORKDIR}"
export PASM_PATH = "${S}/../../../cortexa8hf-neon-poky-linux-gnueabi/pasm-compiler/1.0-r0/git/pru_sw/utils/"

#FILES_${PN} += " \
#	/lib/modules/	\
#	/lib/modules/*	\
#"

#do_install_append() {
#        install -m 0644 hcsr04_FR.bin ${D}/lib/modules/hcsr04_FR.bin
#        install -m 0644 hcsr04_FC.bin ${D}/lib/modules/hcsr04_FR.bin
#        install -m 0644 hcsr04_FL.bin ${D}/lib/modules/hcsr04_FR.bin
#        install -m 0644 hcsr04_RR.bin ${D}/lib/modules/hcsr04_FR.bin
#        install -m 0644 hcsr04_RL.bin ${D}/lib/modules/hcsr04_FR.bin
#}

#do_install_append() {
#    install -d ${D}${bindir}
#    install -m 0755 ${S}/hcsr04_FR.bin ${D}/lib/modules
#}

#do_install() {
#    install -m 0644 hcsr04_FR.bin ${S}/lib/modules
#}

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
