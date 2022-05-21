PART_NAME=firmware
REQUIRE_IMAGE_METADATA=1

RAMFS_COPY_BIN='fw_printenv fw_setenv'
RAMFS_COPY_DATA='/etc/fw_env.config /var/lock/fw_printenv.lock'

platform_check_image() {
	return 0;
}

platform_do_upgrade_xiaomi_nand() {
	local fw_mtd=$(find_mtd_part kernel)
	fw_mtd="${fw_mtd/block/}"
	[ -n "$fw_mtd" ] || return

	local board_dir=$(tar tf "$1" | grep -m 1 '^sysupgrade-.*/$')
	board_dir=${board_dir%/}
	[ -n "$board_dir" ] || return

	local kernel_len=$(tar xf "$1" ${board_dir}/kernel -O | wc -c)
	[ -n "$kernel_len" ] || return

	tar xf "$1" ${board_dir}/kernel -O | ubiformat "$fw_mtd" -y -S $kernel_len -f -

	CI_KERNPART="none"
	nand_do_upgrade "$1"
}

platform_do_upgrade() {
	case "$(board_name)" in
	dynalink,dl-wrx36)
		nand_do_upgrade "$1"
		;;
	edgecore,eap102)
		active="$(fw_printenv -n active)"
		if [ "$active" -eq "1" ]; then
			CI_UBIPART="rootfs2"
		else
			CI_UBIPART="rootfs1"
		fi
		# force altbootcmd which handles partition change in u-boot
		fw_setenv bootcount 3
		fw_setenv upgrade_available 1
		nand_do_upgrade "$1"
		;;
	edimax,cax1800)
		nand_do_upgrade "$1"
		;;
	qnap,301w)
		kernelname="0:HLOS"
		rootfsname="rootfs"
		mmc_do_upgrade "$1"
		;;
	redmi,ax6|\
	xiaomi,ax3600)
		# Enforce single partition.
		fw_setenv flag_boot_rootfs 0
		fw_setenv flag_last_success 0
		fw_setenv flag_boot_success 0
		fw_setenv flag_try_sys1_failed 0

		# Second partition won't work and can't be used
		# Flag is as failed by default
		fw_setenv flag_try_sys2_failed 1

		# kernel and rootfs are placed on 2 different ubi
		# First ubiformat the kernel partition than do nand upgrade
		platform_do_upgrade_xiaomi_nand "$1"
		;;
	xiaomi,ax9000)
		part_num="$(fw_printenv -n flag_boot_rootfs)"
		if [ "$part_num" -eq "0" ]; then
			CI_UBIPART="rootfs_1"
			target_num=1
			# Reset fail flag for the current partition
			# With both partition set to fail, the partition 2 (bit 1)
			# is loaded
			fw_setenv flag_try_sys2_failed 0
		else
			CI_UBIPART="rootfs"
			target_num=0
			# Reset fail flag for the current partition
			# or uboot will skip the loading of this partition
			fw_setenv flag_try_sys1_failed 0
		fi

		# Tell uboot to switch partition
		fw_setenv flag_boot_rootfs $target_num
		fw_setenv flag_last_success $target_num

		# Reset success flag
		fw_setenv flag_boot_success 0

		nand_do_upgrade "$1"
		;;
	*)
		default_do_upgrade "$1"
		;;
	esac
}
