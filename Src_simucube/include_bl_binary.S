		.section .blbin,"ax",%progbits
        .global blbin_start
        .type blbin_start, %object
        .align 4
blbin_start:
        .align 4
        .incbin "../../ion_release_bootload_rom.gdf"

        .global blbin_end
blbin_end:
