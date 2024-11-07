// SPDX-License-Identifier: GPL-2.0-only
/*
 * Airoha AN8855 DSA Switch driver
 * Copyright (C) 2023 Min Yao <min.yao@airoha.com>
 * Copyright (C) 2024 Christian Marangi <ansuelsmth@gmail.com>
 */
#include <linux/bitfield.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/gpio/consumer.h>
#include <linux/if_bridge.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/nvmem-provider.h>
#include <linux/phylink.h>
#include <linux/regmap.h>
#include <net/dsa.h>

#include "an8855.h"

static const struct an8855_mib_desc an8855_mib[] = {
	MIB_DESC(1, 0x00, "TxDrop"),
	MIB_DESC(1, 0x04, "TxCrcErr"),
	MIB_DESC(1, 0x08, "TxUnicast"),
	MIB_DESC(1, 0x0c, "TxMulticast"),
	MIB_DESC(1, 0x10, "TxBroadcast"),
	MIB_DESC(1, 0x14, "TxCollision"),
	MIB_DESC(1, 0x18, "TxSingleCollision"),
	MIB_DESC(1, 0x1c, "TxMultipleCollision"),
	MIB_DESC(1, 0x20, "TxDeferred"),
	MIB_DESC(1, 0x24, "TxLateCollision"),
	MIB_DESC(1, 0x28, "TxExcessiveCollistion"),
	MIB_DESC(1, 0x2c, "TxPause"),
	MIB_DESC(1, 0x30, "TxPktSz64"),
	MIB_DESC(1, 0x34, "TxPktSz65To127"),
	MIB_DESC(1, 0x38, "TxPktSz128To255"),
	MIB_DESC(1, 0x3c, "TxPktSz256To511"),
	MIB_DESC(1, 0x40, "TxPktSz512To1023"),
	MIB_DESC(1, 0x44, "TxPktSz1024To1518"),
	MIB_DESC(1, 0x48, "TxPktSz1519ToMax"),
	MIB_DESC(2, 0x4c, "TxBytes"),
	MIB_DESC(1, 0x54, "TxOversizeDrop"),
	MIB_DESC(2, 0x58, "TxBadPktBytes"),
	MIB_DESC(1, 0x80, "RxDrop"),
	MIB_DESC(1, 0x84, "RxFiltering"),
	MIB_DESC(1, 0x88, "RxUnicast"),
	MIB_DESC(1, 0x8c, "RxMulticast"),
	MIB_DESC(1, 0x90, "RxBroadcast"),
	MIB_DESC(1, 0x94, "RxAlignErr"),
	MIB_DESC(1, 0x98, "RxCrcErr"),
	MIB_DESC(1, 0x9c, "RxUnderSizeErr"),
	MIB_DESC(1, 0xa0, "RxFragErr"),
	MIB_DESC(1, 0xa4, "RxOverSzErr"),
	MIB_DESC(1, 0xa8, "RxJabberErr"),
	MIB_DESC(1, 0xac, "RxPause"),
	MIB_DESC(1, 0xb0, "RxPktSz64"),
	MIB_DESC(1, 0xb4, "RxPktSz65To127"),
	MIB_DESC(1, 0xb8, "RxPktSz128To255"),
	MIB_DESC(1, 0xbc, "RxPktSz256To511"),
	MIB_DESC(1, 0xc0, "RxPktSz512To1023"),
	MIB_DESC(1, 0xc4, "RxPktSz1024To1518"),
	MIB_DESC(1, 0xc8, "RxPktSz1519ToMax"),
	MIB_DESC(2, 0xcc, "RxBytes"),
	MIB_DESC(1, 0xd4, "RxCtrlDrop"),
	MIB_DESC(1, 0xd8, "RxIngressDrop"),
	MIB_DESC(1, 0xdc, "RxArlDrop"),
	MIB_DESC(1, 0xe0, "FlowControlDrop"),
	MIB_DESC(1, 0xe4, "WredDrop"),
	MIB_DESC(1, 0xe8, "MirrorDrop"),
	MIB_DESC(2, 0xec, "RxBadPktBytes"),
	MIB_DESC(1, 0xf4, "RxsFlowSamplingPktDrop"),
	MIB_DESC(1, 0xf8, "RxsFlowTotalPktDrop"),
	MIB_DESC(1, 0xfc, "PortControlDrop"),
};

static int an8855_mii_set_page(struct mii_bus *bus, u8 phy_id, u8 page)
{
	int ret;

	ret = __mdiobus_write(bus, phy_id, AN8855_PHY_SELECT_PAGE, page);
	if (ret < 0)
		dev_err_ratelimited(&bus->dev,
				    "failed to set an8855 mii page\n");

	return ret;
}

static int an8855_mii_read32(struct mii_bus *bus, u8 phy_id, u32 reg, u32 *val)
{
	int lo, hi, ret;

	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_MODE,
			      AN8855_PBUS_MODE_ADDR_FIXED);
	if (ret < 0)
		goto err;

	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_RD_ADDR_HIGH,
			      upper_16_bits(reg));
	if (ret < 0)
		goto err;
	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_RD_ADDR_LOW,
			      lower_16_bits(reg));
	if (ret < 0)
		goto err;

	hi = __mdiobus_read(bus, phy_id, AN8855_PBUS_RD_DATA_HIGH);
	if (hi < 0) {
		ret = hi;
		goto err;
	}
	lo = __mdiobus_read(bus, phy_id, AN8855_PBUS_RD_DATA_LOW);
	if (lo < 0) {
		ret = lo;
		goto err;
	}

	*val = ((u16)hi << 16) | ((u16)lo & 0xffff);

	return 0;
err:
	dev_err_ratelimited(&bus->dev,
			    "failed to read an8855 register\n");
	return ret;
}

static int an8855_regmap_read(void *ctx, uint32_t reg, uint32_t *val)
{
	struct an8855_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);
	ret = an8855_mii_set_page(bus, priv->phy_base, AN8855_PHY_PAGE_EXTENDED_4);
	if (ret < 0)
		goto exit;

	ret = an8855_mii_read32(bus, priv->phy_base,
				reg, val);

exit:
	an8855_mii_set_page(bus, priv->phy_base, AN8855_PHY_PAGE_STANDARD);
	mutex_unlock(&bus->mdio_lock);

	return ret < 0 ? ret : 0;
}

static int an8855_mii_write32(struct mii_bus *bus, u8 phy_id, u32 reg, u32 val)
{
	int ret;

	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_MODE,
			      AN8855_PBUS_MODE_ADDR_FIXED);
	if (ret < 0)
		goto err;

	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_WR_ADDR_HIGH,
			      upper_16_bits(reg));
	if (ret < 0)
		goto err;
	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_WR_ADDR_LOW,
			      lower_16_bits(reg));
	if (ret < 0)
		goto err;

	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_WR_DATA_HIGH,
			      upper_16_bits(val));
	if (ret < 0)
		goto err;
	ret = __mdiobus_write(bus, phy_id, AN8855_PBUS_WR_DATA_LOW,
			      lower_16_bits(val));
	if (ret < 0)
		goto err;

	return 0;
err:
	dev_err_ratelimited(&bus->dev,
			    "failed to write an8855 register\n");
	return ret;
}

static int
an8855_regmap_write(void *ctx, uint32_t reg, uint32_t val)
{
	struct an8855_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);
	ret = an8855_mii_set_page(bus, priv->phy_base, AN8855_PHY_PAGE_EXTENDED_4);
	if (ret < 0)
		goto exit;

	ret = an8855_mii_write32(priv->bus, priv->phy_base,
				 reg, val);

exit:
	an8855_mii_set_page(bus, priv->phy_base, AN8855_PHY_PAGE_STANDARD);
	mutex_unlock(&bus->mdio_lock);

	return ret < 0 ? ret : 0;
}

static int
an8855_regmap_update_bits(void *ctx, uint32_t reg, uint32_t mask, uint32_t write_val)
{
	struct an8855_priv *priv = ctx;
	struct mii_bus *bus = priv->bus;
	u32 val;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);
	ret = an8855_mii_set_page(bus, priv->phy_base, AN8855_PHY_PAGE_EXTENDED_4);
	if (ret < 0)
		goto exit;

	ret = an8855_mii_read32(bus, priv->phy_base, reg, &val);
	if (ret < 0)
		goto exit;

	val &= ~mask;
	val |= write_val;
	ret = an8855_mii_write32(bus, priv->phy_base, reg, val);

exit:
	an8855_mii_set_page(bus, priv->phy_base, AN8855_PHY_PAGE_STANDARD);
	mutex_unlock(&bus->mdio_lock);

	return ret < 0 ? ret : 0;
}

static const struct regmap_range an8855_readable_ranges[] = {
	regmap_reg_range(0x10000000, 0x10000fff), /* SCU */
	regmap_reg_range(0x10001000, 0x10001fff), /* RBUS */
	regmap_reg_range(0x10002000, 0x10002fff), /* MCU */
	regmap_reg_range(0x10005000, 0x10005fff), /* SYS SCU */
	regmap_reg_range(0x10007000, 0x10007fff), /* I2C Slave */
	regmap_reg_range(0x10008000, 0x10008fff), /* I2C Master */
	regmap_reg_range(0x10009000, 0x10009fff), /* PDMA */
	regmap_reg_range(0x1000a100, 0x1000a2ff), /* General Purpose Timer */
	regmap_reg_range(0x1000a200, 0x1000a2ff), /* GPU timer */
	regmap_reg_range(0x1000a300, 0x1000a3ff), /* GPIO */
	regmap_reg_range(0x1000a400, 0x1000a5ff), /* EFUSE */
	regmap_reg_range(0x1000c000, 0x1000cfff), /* GDMP CSR */
	regmap_reg_range(0x10010000, 0x1001ffff), /* GDMP SRAM */
	regmap_reg_range(0x10200000, 0x10203fff), /* Switch - ARL Global */
	regmap_reg_range(0x10204000, 0x10207fff), /* Switch - BMU */
	regmap_reg_range(0x10208000, 0x1020bfff), /* Switch - ARL Port */
	regmap_reg_range(0x1020c000, 0x1020cfff), /* Switch - SCH */
	regmap_reg_range(0x10210000, 0x10213fff), /* Switch - MAC */
	regmap_reg_range(0x10214000, 0x10217fff), /* Switch - MIB */
	regmap_reg_range(0x10218000, 0x1021bfff), /* Switch - Port Control */
	regmap_reg_range(0x1021c000, 0x1021ffff), /* Switch - TOP */
	regmap_reg_range(0x10220000, 0x1022ffff), /* SerDes */
	regmap_reg_range(0x10286000, 0x10286fff), /* RG Batcher */
	regmap_reg_range(0x1028c000, 0x1028ffff), /* ETHER_SYS */
	regmap_reg_range(0x30000000, 0x37ffffff), /* I2C EEPROM */
	regmap_reg_range(0x38000000, 0x3fffffff), /* BOOT_ROM */
	regmap_reg_range(0xa0000000, 0xbfffffff), /* GPHY */
};

const struct regmap_access_table an8855_readable_table = {
	.yes_ranges = an8855_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(an8855_readable_ranges),
};

static const struct regmap_config an8855_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xbfffffff,
	.reg_read = an8855_regmap_read,
	.reg_write = an8855_regmap_write,
	.reg_update_bits = an8855_regmap_update_bits,
	.disable_locking = true,
	.rd_table = &an8855_readable_table,
};

static int
an8855_mib_init(struct an8855_priv *priv)
{
	int ret;

	ret = regmap_write(priv->regmap, AN8855_MIB_CCR, AN8855_CCR_MIB_ENABLE);
	if (ret)
		return ret;

	return regmap_write(priv->regmap, AN8855_MIB_CCR, AN8855_CCR_MIB_ACTIVATE);
}

static void an8855_fdb_write(struct an8855_priv *priv, u16 vid,
			     u8 port_mask, const u8 *mac, bool add)
{
	u32 mac_reg[2] = { };
	u32 reg;

	mac_reg[0] |= FIELD_PREP(AN8855_ATA1_MAC0, mac[0]);
	mac_reg[0] |= FIELD_PREP(AN8855_ATA1_MAC1, mac[1]);
	mac_reg[0] |= FIELD_PREP(AN8855_ATA1_MAC2, mac[2]);
	mac_reg[0] |= FIELD_PREP(AN8855_ATA1_MAC3, mac[3]);
	mac_reg[1] |= FIELD_PREP(AN8855_ATA2_MAC4, mac[4]);
	mac_reg[1] |= FIELD_PREP(AN8855_ATA2_MAC5, mac[5]);

	regmap_bulk_write(priv->regmap, AN8855_ATA1, mac_reg,
			  ARRAY_SIZE(mac_reg));

	reg = AN8855_ATWD_IVL;
	if (add)
		reg |= AN8855_ATWD_VLD;
	reg |= FIELD_PREP(AN8855_ATWD_VID, vid);
	regmap_write(priv->regmap, AN8855_ATWD, reg);
	regmap_write(priv->regmap, AN8855_ATWD2,
		     FIELD_PREP(AN8855_ATWD2_PORT, port_mask));
}

static void an8855_fdb_read(struct an8855_priv *priv, struct an8855_fdb *fdb)
{
	u32 reg[4];

	regmap_bulk_read(priv->regmap, AN8855_ATRD0, reg,
			 ARRAY_SIZE(reg));

	fdb->live = FIELD_GET(AN8855_ATRD0_LIVE, reg[0]);
	fdb->type = FIELD_GET(AN8855_ATRD0_TYPE, reg[0]);
	fdb->ivl = FIELD_GET(AN8855_ATRD0_IVL, reg[0]);
	fdb->vid = FIELD_GET(AN8855_ATRD0_VID, reg[0]);
	fdb->fid = FIELD_GET(AN8855_ATRD0_FID, reg[0]);
	fdb->aging = FIELD_GET(AN8855_ATRD1_AGING, reg[1]);
	fdb->port_mask = FIELD_GET(AN8855_ATRD3_PORTMASK, reg[3]);
	fdb->mac[0] = FIELD_GET(AN8855_ATRD2_MAC0, reg[2]);
	fdb->mac[1] = FIELD_GET(AN8855_ATRD2_MAC1, reg[2]);
	fdb->mac[2] = FIELD_GET(AN8855_ATRD2_MAC2, reg[2]);
	fdb->mac[3] = FIELD_GET(AN8855_ATRD2_MAC3, reg[2]);
	fdb->mac[4] = FIELD_GET(AN8855_ATRD1_MAC4, reg[1]);
	fdb->mac[5] = FIELD_GET(AN8855_ATRD1_MAC5, reg[1]);
	fdb->noarp = !!FIELD_GET(AN8855_ATRD0_ARP, reg[0]);
}

static int an8855_fdb_cmd(struct an8855_priv *priv, u32 cmd, u32 *rsp)
{
	u32 val;
	int ret;

	/* Set the command operating upon the MAC address entries */
	val = AN8855_ATC_BUSY | cmd;
	ret = regmap_write(priv->regmap, AN8855_ATC, val);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(priv->regmap, AN8855_ATC, val,
				       !(val & AN8855_ATC_BUSY), 20, 200000);
	if (ret)
		return ret;

	if (rsp)
		*rsp = val;

	return 0;
}

static void
an8855_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct an8855_priv *priv = ds->priv;
	u32 stp_state;

	switch (state) {
	case BR_STATE_DISABLED:
		stp_state = AN8855_STP_DISABLED;
		break;
	case BR_STATE_BLOCKING:
		stp_state = AN8855_STP_BLOCKING;
		break;
	case BR_STATE_LISTENING:
		stp_state = AN8855_STP_LISTENING;
		break;
	case BR_STATE_LEARNING:
		stp_state = AN8855_STP_LEARNING;
		break;
	case BR_STATE_FORWARDING:
	default:
		stp_state = AN8855_STP_FORWARDING;
		break;
	}

	regmap_update_bits(priv->regmap, AN8855_SSP_P(port), AN8855_FID_PST,
			   stp_state);
}

static int an8855_port_bridge_join(struct dsa_switch *ds, int port,
				   struct dsa_bridge bridge,
				   bool *tx_fwd_offload,
				   struct netlink_ext_ack *extack)
{
	struct an8855_priv *priv = ds->priv;
	u32 port_mask = BIT(AN8855_CPU_PORT);
	struct dsa_port *dp;
	int ret;

	dsa_switch_for_each_port(dp, ds) {
		if (dp->index == port)
			continue;

		if (dsa_port_is_cpu(dp))
			continue;

		if (!dsa_port_offloads_bridge_dev(dp, bridge.dev))
			continue;

		/* Add this port to the portvlan mask of the other
		 * ports in the bridge
		 */
		port_mask |= BIT(dp->index);
		ret = regmap_set_bits(priv->regmap, AN8855_PORTMATRIX_P(dp->index),
				      FIELD_PREP(AN8855_PORTMATRIX, port));
		if (ret)
			return ret;
	}

	/* Add all other ports to this port's portvlan mask */
	return regmap_update_bits(priv->regmap, AN8855_PORTMATRIX_P(port),
				  AN8855_PORTMATRIX, port_mask);
}

static void an8855_port_bridge_leave(struct dsa_switch *ds, int port,
				     struct dsa_bridge bridge)
{
	struct an8855_priv *priv = ds->priv;
	struct dsa_port *dp;
	u32 port_mask = 0;

	dsa_switch_for_each_port(dp, ds) {
		if (dp->index == port)
			continue;

		if (dsa_port_is_cpu(dp))
			continue;

		if (!dsa_port_offloads_bridge_dev(dp, bridge.dev))
			continue;

		/* Remove this port from the portvlan mask of the other
		 * ports in the bridge
		 */
		port_mask |= BIT(dp->index);
		regmap_clear_bits(priv->regmap, AN8855_PORTMATRIX_P(dp->index),
				  FIELD_PREP(AN8855_PORTMATRIX, port));
	}

	/* Remove all other ports from this port's portvlan mask */
	regmap_update_bits(priv->regmap, AN8855_PORTMATRIX_P(port),
			   AN8855_PORTMATRIX,
			   FIELD_PREP(AN8855_PORTMATRIX, ~port_mask));
}

static int an8855_port_fdb_add(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid,
			       struct dsa_db db)
{
	struct an8855_priv *priv = ds->priv;
	u8 port_mask = BIT(port);
	int ret;

	mutex_lock(&priv->reg_mutex);
	an8855_fdb_write(priv, vid, port_mask, addr, 1);
	ret = an8855_fdb_cmd(priv, AN8855_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int an8855_port_fdb_del(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid,
			       struct dsa_db db)
{
	struct an8855_priv *priv = ds->priv;
	u8 port_mask = BIT(port);
	int ret;

	mutex_lock(&priv->reg_mutex);
	an8855_fdb_write(priv, vid, port_mask, addr, 0);
	ret = an8855_fdb_cmd(priv, AN8855_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);

	return ret;
}

static int an8855_port_fdb_dump(struct dsa_switch *ds, int port,
				dsa_fdb_dump_cb_t *cb, void *data)
{
	struct an8855_priv *priv = ds->priv;
	struct an8855_fdb _fdb = {  };
	int banks, count = 0;
	u32 rsp;
	int ret;
	int i;

	mutex_lock(&priv->reg_mutex);

	/* Load search port */
	ret = regmap_write(priv->regmap, AN8855_ATWD2,
			   FIELD_PREP(AN8855_ATWD2_PORT, port));
	if (ret)
		goto exit;
	ret = an8855_fdb_cmd(priv, AN8855_ATC_MAT(AND8855_FDB_MAT_MAC_PORT) |
			     AN8855_FDB_START, &rsp);
	if (ret < 0)
		goto exit;

	do {
		/* From response get the number of banks to read, exit if 0 */
		banks = FIELD_GET(AN8855_ATC_HIT, rsp);
		if (!banks)
			break;

		/* Each banks have 4 entry */
		for (i = 0; i < 4; i++) {
			count++;

			/* Check if bank is present */
			if (!(banks & BIT(i)))
				continue;

			/* Select bank entry index */
			ret = regmap_write(priv->regmap, AN8855_ATRDS,
					   FIELD_PREP(AN8855_ATRD_SEL, i));
			if (ret)
				break;
			/* wait 1ms for the bank entry to be filled */
			usleep_range(1000, 1500);
			an8855_fdb_read(priv, &_fdb);

			if (!_fdb.live)
				continue;
			ret = cb(_fdb.mac, _fdb.vid, _fdb.noarp, data);
			if (ret < 0)
				break;
		}

		/* Stop if reached max FDB number */
		if (count >= AN8855_NUM_FDB_RECORDS)
			break;

		/* Read next bank */
		ret = an8855_fdb_cmd(priv, AN8855_ATC_MAT(AND8855_FDB_MAT_MAC_PORT) |
				     AN8855_FDB_NEXT, &rsp);
		if (ret < 0)
			break;
	} while (true);

exit:
	mutex_unlock(&priv->reg_mutex);
	return ret;
}

static int an8855_vlan_cmd(struct an8855_priv *priv, enum an8855_vlan_cmd cmd,
			   u16 vid)
{
	u32 val;
	int ret;

	val = AN8855_VTCR_BUSY | FIELD_PREP(AN8855_VTCR_FUNC, cmd) |
	      FIELD_PREP(AN8855_VTCR_VID, vid);
	ret = regmap_write(priv->regmap, AN8855_VTCR, val);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(priv->regmap, AN8855_VTCR, val,
					!(val & AN8855_VTCR_BUSY), 20, 200000);
}

static int an8855_vlan_add(struct an8855_priv *priv, u8 port, u16 vid,
			   bool untagged)
{
	u32 port_mask;
	u32 val;
	int ret;

	/* Fetch entry */
	ret = an8855_vlan_cmd(priv, AN8855_VTCR_RD_VID, vid);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, AN8855_VARD0, &val);
	if (ret)
		return ret;
	port_mask = FIELD_GET(AN8855_VA0_PORT, val) | BIT(port);

	/* Validate the entry with independent learning, create egress tag per
	 * VLAN and joining the port as one of the port members.
	 */
	val = (val & AN8855_VA0_ETAG) | AN8855_VA0_IVL_MAC |
	      AN8855_VA0_VTAG_EN | AN8855_VA0_VLAN_VALID |
	      FIELD_PREP(AN8855_VA0_PORT, port_mask);
	ret = regmap_write(priv->regmap, AN8855_VAWD0, val);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, AN8855_VAWD1, 0);
	if (ret)
		return ret;

	/* CPU port is always taken as a tagged port for serving more than one
	 * VLANs across and also being applied with egress type stack mode for
	 * that VLAN tags would be appended after hardware special tag used as
	 * DSA tag.
	 */
	if (port == AN8855_CPU_PORT)
		val = AN8855_VLAN_EGRESS_STACK;
	/* Decide whether adding tag or not for those outgoing packets from the
	 * port inside the VLAN.
	 */
	else
		val = untagged ? AN8855_VLAN_EGRESS_UNTAG : AN8855_VLAN_EGRESS_TAG;
	ret = regmap_update_bits(priv->regmap, AN8855_VAWD0,
				 AN8855_VA0_ETAG_PORT_MASK(port),
				 AN8855_VA0_ETAG_PORT_VAL(port, val));
	if (ret)
		return ret;

	/* Flush result to hardware */
	return an8855_vlan_cmd(priv, AN8855_VTCR_WR_VID, vid);
}

static int an8855_vlan_del(struct an8855_priv *priv, u8 port, u16 vid)
{
	u32 port_mask;
	u32 val;
	int ret;

	/* Fetch entry */
	ret = an8855_vlan_cmd(priv, AN8855_VTCR_RD_VID, vid);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, AN8855_VARD0, &val);
	if (ret)
		return ret;
	port_mask = FIELD_GET(AN8855_VA0_PORT, val) & ~BIT(port);

	if (!(val & AN8855_VA0_VLAN_VALID)) {
		dev_err(priv->dev, "Cannot be deleted due to invalid entry\n");
		return -EINVAL;
	}

	if (port_mask) {
		val = (val & AN8855_VA0_ETAG) | AN8855_VA0_IVL_MAC |
		       AN8855_VA0_VTAG_EN | AN8855_VA0_VLAN_VALID |
		       FIELD_PREP(AN8855_VA0_PORT, port_mask);
		ret = regmap_write(priv->regmap, AN8855_VAWD0, val);
		if (ret)
			return ret;
	} else {
		ret = regmap_write(priv->regmap, AN8855_VAWD0, 0);
		if (ret)
			return ret;
	}
	ret = regmap_write(priv->regmap, AN8855_VAWD1, 0);
	if (ret)
		return ret;

	/* Flush result to hardware */
	return an8855_vlan_cmd(priv, AN8855_VTCR_WR_VID, vid);
}

static int an8855_port_set_vlan_mode(struct an8855_priv *priv, int port,
				     enum an8855_port_mode port_mode,
				     enum an8855_vlan_port_eg_tag eg_tag,
				     enum an8855_vlan_port_attr vlan_attr)
{
	int ret;

	ret = regmap_update_bits(priv->regmap, AN8855_PCR_P(port),
				 AN8855_PORT_VLAN,
				 FIELD_PREP(AN8855_PORT_VLAN, port_mode));
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, AN8855_PVC_P(port),
				  AN8855_PVC_EG_TAG | AN8855_VLAN_ATTR,
				  FIELD_PREP(AN8855_PVC_EG_TAG, eg_tag) |
				  FIELD_PREP(AN8855_VLAN_ATTR, vlan_attr));
}

static int an8855_port_vlan_filtering(struct dsa_switch *ds, int port,
				      bool vlan_filtering,
				      struct netlink_ext_ack *extack)
{
	struct an8855_priv *priv = ds->priv;
	int ret;

	/* The port is being kept as VLAN-unaware port when bridge is
	 * set up with vlan_filtering not being set, Otherwise, the
	 * port and the corresponding CPU port is required the setup
	 * for becoming a VLAN-aware port.
	 */
	if (vlan_filtering) {
		/* CPU port is set to fallback mode to let untagged
		 * frames pass through.
		 */
		ret = an8855_port_set_vlan_mode(priv, AN8855_CPU_PORT,
						AN8855_PORT_FALLBACK_MODE,
						AN8855_VLAN_EG_DISABLED,
						AN8855_VLAN_USER);
		if (ret)
			return ret;

		/* Trapped into security mode allows packet forwarding through VLAN
		 * table lookup.
		 * Set the port as a user port which is to be able to recognize VID
		 * from incoming packets before fetching entry within the VLAN table.
		 */
		ret = an8855_port_set_vlan_mode(priv, port,
						AN8855_PORT_SECURITY_MODE,
						AN8855_VLAN_EG_DISABLED,
						AN8855_VLAN_USER);
		if (ret)
			return ret;
	} else {
		bool disable_cpu_vlan = true;
		struct dsa_port *dp;

		/* When a port is removed from the bridge, the port would be set up
		 * back to the default as is at initial boot which is a VLAN-unaware
		 * port.
		 */
		ret = an8855_port_set_vlan_mode(priv, port, AN8855_PORT_MATRIX_MODE,
						AN8855_VLAN_EG_CONSISTENT,
						AN8855_VLAN_TRANSPARENT);
		if (ret)
			return ret;

		dsa_switch_for_each_user_port(dp, ds) {
			if (dsa_port_is_vlan_filtering(dp)) {
				disable_cpu_vlan = false;
				break;
			}
		}

		if (disable_cpu_vlan) {
			ret = an8855_port_set_vlan_mode(priv, AN8855_CPU_PORT,
							AN8855_PORT_MATRIX_MODE,
							AN8855_VLAN_EG_CONSISTENT,
							AN8855_VLAN_TRANSPARENT);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int an8855_port_vlan_add(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan,
				struct netlink_ext_ack *extack)
{
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	struct an8855_priv *priv = ds->priv;
	int ret;

	mutex_lock(&priv->reg_mutex);
	ret = an8855_vlan_add(priv, port, vlan->vid, untagged);
	mutex_unlock(&priv->reg_mutex);
	if (ret)
		return ret;

	if (pvid) {
		ret = regmap_update_bits(priv->regmap, AN8855_PVID_P(port),
					 AN8855_G0_PORT_VID,
					 FIELD_PREP(AN8855_G0_PORT_VID, vlan->vid));
		if (ret)
			return ret;
	}

	return 0;
}

static int an8855_port_vlan_del(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
	struct an8855_priv *priv = ds->priv;
	u32 val;
	int ret;

	mutex_lock(&priv->reg_mutex);
	ret = an8855_vlan_del(priv, port, vlan->vid);
	mutex_unlock(&priv->reg_mutex);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, AN8855_PVID_P(port), &val);
	if (ret)
		return ret;
	if (FIELD_GET(AN8855_G0_PORT_VID, val) == vlan->vid) {
		ret = regmap_update_bits(priv->regmap, AN8855_PVID_P(port),
					 AN8855_G0_PORT_VID,
					 FIELD_PREP(AN8855_G0_PORT_VID,
						    AN8855_PORT_VID_DEFAULT));
		if (ret)
			return ret;
	}

	return 0;
}

static void
an8855_get_strings(struct dsa_switch *ds, int port, u32 stringset,
		   uint8_t *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(an8855_mib); i++)
		ethtool_puts(&data, an8855_mib[i].name);
}

static void
an8855_get_ethtool_stats(struct dsa_switch *ds, int port, uint64_t *data)
{
	struct an8855_priv *priv = ds->priv;
	const struct an8855_mib_desc *mib;
	u32 reg, i, val;
	u32 hi;

	for (i = 0; i < ARRAY_SIZE(an8855_mib); i++) {
		mib = &an8855_mib[i];
		reg = AN8855_PORT_MIB_COUNTER(port) + mib->offset;

		regmap_read(priv->regmap, reg, &val);
		if (mib->size == 2)
			regmap_read(priv->regmap, reg + 4, &hi);

		data[i] = val;
		if (mib->size == 2)
			data[i] |= (u64)hi << 32;
	}
}

static int
an8855_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(an8855_mib);
}

static int an8855_port_mirror_add(struct dsa_switch *ds, int port,
				  struct dsa_mall_mirror_tc_entry *mirror,
				  bool ingress,
				  struct netlink_ext_ack *extack)
{
	struct an8855_priv *priv = ds->priv;
	int monitor_port;
	u32 val;
	int ret;

	/* Check for existent entry */
	if ((ingress ? priv->mirror_rx : priv->mirror_tx) & BIT(port))
		return -EEXIST;

	ret = regmap_read(priv->regmap, AN8855_MIR, &val);
	if (ret)
		return ret;

	/* AN8855 supports 4 monitor port, but only use first group */
	monitor_port = FIELD_GET(AN8855_MIRROR_PORT, val);
	if (val & AN8855_MIRROR_EN && monitor_port != mirror->to_local_port)
		return -EEXIST;

	val = AN8855_MIRROR_EN;
	val |= FIELD_PREP(AN8855_MIRROR_PORT, mirror->to_local_port);
	ret = regmap_update_bits(priv->regmap, AN8855_MIR,
				 AN8855_MIRROR_EN | AN8855_MIRROR_PORT,
				 val);
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, AN8855_PCR_P(port),
			      ingress ? AN8855_PORT_RX_MIR : AN8855_PORT_TX_MIR);
	if (ret)
		return ret;

	if (ingress)
		priv->mirror_rx |= BIT(port);
	else
		priv->mirror_tx |= BIT(port);

	return 0;
}

static void an8855_port_mirror_del(struct dsa_switch *ds, int port,
				   struct dsa_mall_mirror_tc_entry *mirror)
{
	struct an8855_priv *priv = ds->priv;

	if (mirror->ingress)
		priv->mirror_rx &= ~BIT(port);
	else
		priv->mirror_tx &= ~BIT(port);

	regmap_clear_bits(priv->regmap, AN8855_PCR_P(port),
			  mirror->ingress ? AN8855_PORT_RX_MIR :
					    AN8855_PORT_TX_MIR);

	if (!priv->mirror_rx && !priv->mirror_tx)
		regmap_clear_bits(priv->regmap, AN8855_MIR, AN8855_MIRROR_EN);
}

static int an8855_port_set_status(struct an8855_priv *priv, int port,
				  bool enable)
{
	if (enable)
		return regmap_set_bits(priv->regmap, AN8855_PMCR_P(port),
				       AN8855_PMCR_TX_EN | AN8855_PMCR_RX_EN);
	else
		return regmap_clear_bits(priv->regmap, AN8855_PMCR_P(port),
					 AN8855_PMCR_TX_EN | AN8855_PMCR_RX_EN);
}

static int an8855_port_enable(struct dsa_switch *ds, int port,
			      struct phy_device *phy)
{
	int ret;

	ret = an8855_port_set_status(ds->priv, port, true);
	if (ret)
		return ret;

	if (dsa_is_user_port(ds, port))
		phy_support_asym_pause(phy);

	return 0;
}

static void an8855_port_disable(struct dsa_switch *ds, int port)
{
	an8855_port_set_status(ds->priv, port, false);
}

static int an8855_set_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_eee *eee)
{
	struct an8855_priv *priv = ds->priv;
	u32 reg;
	int ret;

	if (eee->eee_enabled) {
		ret = regmap_read(priv->regmap, AN8855_PMCR_P(port), &reg);
		if (ret)
			return ret;
		/* Force enable EEE if force mode and LINK */
		if (reg & AN8855_PMCR_FORCE_MODE &&
		    reg & AN8855_PMCR_FORCE_LNK) {
			switch (reg & AN8855_PMCR_FORCE_SPEED) {
			case AN8855_PMCR_FORCE_SPEED_1000:
				reg |= AN8855_PMCR_FORCE_EEE1G;
				break;
			case AN8855_PMCR_FORCE_SPEED_100:
				reg |= AN8855_PMCR_FORCE_EEE100;
				break;
			default:
				break;
			}
			ret = regmap_write(priv->regmap, AN8855_PMCR_P(port), reg);
			if (ret)
				return ret;
		}
		ret = regmap_update_bits(priv->regmap, AN8855_PMEEECR_P(port),
					 AN8855_LPI_MODE_EN,
					 eee->tx_lpi_enabled ? AN8855_LPI_MODE_EN : 0);
		if (ret)
			return ret;
	} else {
		ret = regmap_clear_bits(priv->regmap, AN8855_PMCR_P(port),
					AN8855_PMCR_FORCE_EEE1G |
					AN8855_PMCR_FORCE_EEE100);
		if (ret)
			return ret;

		ret = regmap_clear_bits(priv->regmap, AN8855_PMEEECR_P(port),
					AN8855_LPI_MODE_EN);
		if (ret)
			return ret;
	}

	return 0;
}

static int an8855_get_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_eee *eee)
{
	struct an8855_priv *priv = ds->priv;
	u32 reg;
	int ret;

	ret = regmap_read(priv->regmap, AN8855_PMEEECR_P(port), &reg);
	if (ret)
		return ret;
	eee->tx_lpi_enabled = reg & AN8855_LPI_MODE_EN;

	ret = regmap_read(priv->regmap, AN8855_CKGCR, &reg);
	if (ret)
		return ret;
	/* Global LPI TXIDLE Threshold, default 60ms (unit 2us) */
	eee->tx_lpi_timer = FIELD_GET(AN8855_LPI_TXIDLE_THD_MASK, reg) / 500;

	ret = regmap_read(priv->regmap, AN8855_PMSR_P(port), &reg);
	if (ret)
		return ret;

	return 0;
}

static u32 en8855_get_phy_flags(struct dsa_switch *ds, int port)
{
	struct an8855_priv *priv = ds->priv;

	/* PHY doesn't need calibration */
	if (!priv->phy_require_calib)
		return 0;

	/* Use BIT(0) to signal calibration needed */
	return BIT(0);
}

static enum dsa_tag_protocol
an8855_get_tag_protocol(struct dsa_switch *ds, int port,
			enum dsa_tag_protocol mp)
{
	return DSA_TAG_PROTO_MTK;
}

static int an8855_phy_read(struct mii_bus *bus, int phy, int regnum)
{
	struct an8855_priv *priv = bus->priv;

	return mdiobus_read_nested(priv->bus, phy, regnum);
}

static int an8855_phy_write(struct mii_bus *bus, int phy, int regnum, u16 val)
{
	struct an8855_priv *priv = bus->priv;

	return mdiobus_write_nested(priv->bus, phy, regnum, val);
}

static int an8855_mdio_setup(struct an8855_priv *priv)
{
	struct dsa_switch *ds = priv->ds;
	struct device *dev = priv->dev;
	struct device_node *np;
	struct mii_bus *bus;
	int ret = 0;

	np = of_get_child_by_name(priv->dev->of_node, "mdio");
	if (!np || !of_device_is_available(np))
		goto exit;

	bus = devm_mdiobus_alloc(priv->dev);
	if (!bus) {
		ret = -ENOMEM;
		goto exit;
	}

	bus->priv = priv;
	bus->name = KBUILD_MODNAME "-mii";
	snprintf(bus->id, MII_BUS_ID_SIZE, KBUILD_MODNAME "-%d.%d",
		 ds->dst->index, ds->index);
	bus->parent = dev;
	bus->read = an8855_phy_read;
	bus->write = an8855_phy_write;

	ret = devm_of_mdiobus_register(dev, bus, np);
	if (ret)
		dev_err(dev, "failed to register MDIO bus: %d", ret);

exit:
	of_node_put(np);
	return ret;
}

static int an8855_setup(struct dsa_switch *ds)
{
	struct an8855_priv *priv = ds->priv;
	struct dsa_port *dp;
	int ret;

	/* Setup mdio BUS for internal PHY */
	ret = an8855_mdio_setup(priv);
	if (ret)
		return ret;

	/* Enable and reset MIB counters */
	ret = an8855_mib_init(priv);
	if (ret)
		return ret;

	dsa_switch_for_each_port(dp, ds) {
		/* Disable forwarding by default on all ports */
		ret = regmap_clear_bits(priv->regmap, AN8855_PORTMATRIX_P(dp->index),
					AN8855_PORTMATRIX);
		if (ret)
			return ret;

		/* Enable consistent egress tag */
		ret = regmap_update_bits(priv->regmap, AN8855_PVC_P(dp->index),
					 AN8855_PVC_EG_TAG,
					 FIELD_PREP(AN8855_PVC_EG_TAG,
						    AN8855_VLAN_EG_CONSISTENT));
		if (ret)
			return ret;
	}

	/* Disable MAC by default on all user ports */
	dsa_switch_for_each_user_port(dp, ds) {
		ret = an8855_port_set_status(priv, dp->index, false);
		if (ret)
			return ret;
	}

	/* Enable Airoha header mode on the cpu port */
	ret = regmap_write(priv->regmap, AN8855_PVC_P(AN8855_CPU_PORT),
			   AN8855_PORT_SPEC_REPLACE_MODE | AN8855_PORT_SPEC_TAG);
	if (ret)
		return ret;

	/* Unknown multicast frame forwarding to the cpu port */
	ret = regmap_write(priv->regmap, AN8855_UNMF, BIT(AN8855_CPU_PORT));
	if (ret)
		return ret;

	/* Set CPU port number */
	ret = regmap_update_bits(priv->regmap, AN8855_MFC,
				 AN8855_CPU_EN | AN8855_CPU_PORT_IDX,
				 AN8855_CPU_EN |
				 FIELD_PREP(AN8855_CPU_PORT_IDX, AN8855_CPU_PORT));
	if (ret)
		return ret;

	/* CPU port gets connected to all user ports of
	 * the switch.
	 */
	ret = regmap_write(priv->regmap, AN8855_PORTMATRIX_P(AN8855_CPU_PORT),
			   FIELD_PREP(AN8855_PORTMATRIX, dsa_user_ports(ds)));
	if (ret)
		return ret;

	/* BPDU to CPU port */
	ret = regmap_update_bits(priv->regmap, AN8855_BPC, AN8855_BPDU_PORT_FW,
				 FIELD_PREP(AN8855_BPDU_PORT_FW, AN8855_BPDU_CPU_ONLY));
	if (ret)
		return ret;

	ret = regmap_clear_bits(priv->regmap, AN8855_CKGCR,
				AN8855_CKG_LNKDN_GLB_STOP | AN8855_CKG_LNKDN_PORT_STOP);
	if (ret)
		return ret;

	/* Release global PHY power down */
	ret = regmap_write(priv->regmap, AN8855_RG_GPHY_AFE_PWD, 0x0);
	if (ret)
		return ret;

	ds->configure_vlan_while_not_filtering = true;

	/* Flush the FDB table */
	ret = an8855_fdb_cmd(priv, AN8855_FDB_FLUSH, NULL);
	if (ret < 0)
		return ret;

	return 0;
}

static struct phylink_pcs *
an8855_phylink_mac_select_pcs(struct phylink_config *config,
			      phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct an8855_priv *priv = dp->ds->priv;

	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		return &priv->pcs;
	default:
		return NULL;
	}
}

static void
an8855_phylink_mac_config(struct phylink_config *config, unsigned int mode,
			  const struct phylink_link_state *state)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct dsa_switch *ds = dp->ds;
	struct an8855_priv *priv;
	int port = dp->index;

	priv = ds->priv;

	switch (port) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		return;
	case 5:
		break;
	default:
		dev_err(ds->dev, "unsupported port: %d", port);
		return;
	}

	regmap_update_bits(priv->regmap, AN8855_PMCR_P(port),
			   AN8855_PMCR_IFG_XMIT | AN8855_PMCR_MAC_MODE |
			   AN8855_PMCR_BACKOFF_EN | AN8855_PMCR_BACKPR_EN,
			   FIELD_PREP(AN8855_PMCR_IFG_XMIT, 0x1) |
			   AN8855_PMCR_MAC_MODE | AN8855_PMCR_BACKOFF_EN |
			   AN8855_PMCR_BACKPR_EN);
}

static void an8855_phylink_get_caps(struct dsa_switch *ds, int port,
				    struct phylink_config *config)
{
	switch (port) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		__set_bit(PHY_INTERFACE_MODE_GMII,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		break;
	case 5:
		phy_interface_set_rgmii(config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_SGMII,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_2500BASEX,
			  config->supported_interfaces);
		break;
	}

	config->mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
				   MAC_10 | MAC_100 | MAC_1000FD;
}

static void
an8855_phylink_mac_link_down(struct phylink_config *config, unsigned int mode,
			     phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct an8855_priv *priv = dp->ds->priv;

	/* With autoneg just disable TX/RX else also force link down */
	if (phylink_autoneg_inband(mode)) {
		regmap_clear_bits(priv->regmap, AN8855_PMCR_P(dp->index),
				  AN8855_PMCR_TX_EN | AN8855_PMCR_RX_EN);
	} else {
		regmap_update_bits(priv->regmap, AN8855_PMCR_P(dp->index),
				   AN8855_PMCR_TX_EN | AN8855_PMCR_RX_EN |
				   AN8855_PMCR_FORCE_MODE | AN8855_PMCR_FORCE_LNK,
				   AN8855_PMCR_FORCE_MODE);
	}
}

static void
an8855_phylink_mac_link_up(struct phylink_config *config,
			   struct phy_device *phydev, unsigned int mode,
			   phy_interface_t interface, int speed, int duplex,
			   bool tx_pause, bool rx_pause)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct an8855_priv *priv = dp->ds->priv;
	int port = dp->index;
	u32 reg;

	reg = regmap_read(priv->regmap, AN8855_PMCR_P(port), &reg);
	if (phylink_autoneg_inband(mode)) {
		reg &= ~AN8855_PMCR_FORCE_MODE;
	} else {
		reg |= AN8855_PMCR_FORCE_MODE | AN8855_PMCR_FORCE_LNK;

		reg &= ~AN8855_PMCR_FORCE_SPEED;
		switch (speed) {
		case SPEED_10:
			reg |= AN8855_PMCR_FORCE_SPEED_10;
			break;
		case SPEED_100:
			reg |= AN8855_PMCR_FORCE_SPEED_100;
			break;
		case SPEED_1000:
			reg |= AN8855_PMCR_FORCE_SPEED_1000;
			break;
		case SPEED_2500:
			reg |= AN8855_PMCR_FORCE_SPEED_2500;
			break;
		case SPEED_5000:
			reg |= AN8855_PMCR_FORCE_SPEED_5000;
			break;
		}

		reg &= AN8855_PMCR_FORCE_FDX;
		if (duplex == DUPLEX_FULL)
			reg |= AN8855_PMCR_FORCE_FDX;

		reg &= AN8855_PMCR_RX_FC_EN;
		if (rx_pause || dsa_port_is_cpu(dp))
			reg |= AN8855_PMCR_RX_FC_EN;

		reg &= AN8855_PMCR_TX_FC_EN;
		if (rx_pause || dsa_port_is_cpu(dp))
			reg |= AN8855_PMCR_TX_FC_EN;

		/* Disable any EEE options */
		reg &= ~(AN8855_PMCR_FORCE_EEE5G | AN8855_PMCR_FORCE_EEE2P5G |
			 AN8855_PMCR_FORCE_EEE1G | AN8855_PMCR_FORCE_EEE100);
	}

	reg |= AN8855_PMCR_TX_EN | AN8855_PMCR_RX_EN;

	regmap_write(priv->regmap, AN8855_PMCR_P(port), reg);
}

static void an8855_pcs_get_state(struct phylink_pcs *pcs,
				 struct phylink_link_state *state)
{
	struct an8855_priv *priv = container_of(pcs, struct an8855_priv, pcs);
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, AN8855_PMSR_P(AN8855_CPU_PORT), &val);
	if (ret < 0) {
		state->link = false;
		return;
	}

	state->link = !!(val & AN8855_PMSR_LNK);
	state->an_complete = state->link;
	state->duplex = (val & AN8855_PMSR_DPX) ? DUPLEX_FULL :
						  DUPLEX_HALF;

	switch (val & AN8855_PMSR_SPEED) {
	case AN8855_PMSR_SPEED_10:
		state->speed = SPEED_10;
		break;
	case AN8855_PMSR_SPEED_100:
		state->speed = SPEED_100;
		break;
	case AN8855_PMSR_SPEED_1000:
		state->speed = SPEED_1000;
		break;
	case AN8855_PMSR_SPEED_2500:
		state->speed = SPEED_2500;
		break;
	case AN8855_PMSR_SPEED_5000:
		state->speed = SPEED_5000;
		break;
	default:
		state->speed = SPEED_UNKNOWN;
		break;
	}

	if (val & AN8855_PMSR_RX_FC)
		state->pause |= MLO_PAUSE_RX;
	if (val & AN8855_PMSR_TX_FC)
		state->pause |= MLO_PAUSE_TX;
}

static int an8855_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			     phy_interface_t interface,
			     const unsigned long *advertising,
			     bool permit_pause_to_mac)
{
	struct an8855_priv *priv = container_of(pcs, struct an8855_priv, pcs);
	u32 val;
	int ret;

	switch (interface) {
	case PHY_INTERFACE_MODE_RGMII:
		return 0;
	case PHY_INTERFACE_MODE_SGMII:
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED) {
			dev_err(priv->dev, "in-band negotiation unsupported");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	/*                   !!! WELCOME TO HELL !!!                   */

	/* TX FIR - improve TX EYE */
	ret = regmap_update_bits(priv->regmap, AN8855_INTF_CTRL_10,
				 AN8855_RG_DA_QP_TX_FIR_C2_SEL |
				 AN8855_RG_DA_QP_TX_FIR_C2_FORCE |
				 AN8855_RG_DA_QP_TX_FIR_C1_SEL |
				 AN8855_RG_DA_QP_TX_FIR_C1_FORCE,
				 AN8855_RG_DA_QP_TX_FIR_C2_SEL |
				 FIELD_PREP(AN8855_RG_DA_QP_TX_FIR_C2_FORCE, 0x4) |
				 AN8855_RG_DA_QP_TX_FIR_C1_SEL |
				 FIELD_PREP(AN8855_RG_DA_QP_TX_FIR_C1_FORCE, 0x0));
	if (ret)
		return ret;

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x0;
	else
		val = 0xd;
	ret = regmap_update_bits(priv->regmap, AN8855_INTF_CTRL_11,
				 AN8855_RG_DA_QP_TX_FIR_C0B_SEL |
				 AN8855_RG_DA_QP_TX_FIR_C0B_FORCE,
				 AN8855_RG_DA_QP_TX_FIR_C0B_SEL |
				 FIELD_PREP(AN8855_RG_DA_QP_TX_FIR_C0B_FORCE, val));
	if (ret)
		return ret;

	/* RX CDR - improve RX Jitter Tolerance */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x5;
	else
		val = 0x6;
	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_CDR_LPF_BOT_LIM,
				 AN8855_RG_QP_CDR_LPF_KP_GAIN |
				 AN8855_RG_QP_CDR_LPF_KI_GAIN,
				 FIELD_PREP(AN8855_RG_QP_CDR_LPF_KP_GAIN, val) |
				 FIELD_PREP(AN8855_RG_QP_CDR_LPF_KI_GAIN, val));
	if (ret)
		return ret;

	/* PLL */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x1;
	else
		val = 0x0;
	ret = regmap_update_bits(priv->regmap, AN8855_QP_DIG_MODE_CTRL_1,
				 AN8855_RG_TPHY_SPEED,
				 FIELD_PREP(AN8855_RG_TPHY_SPEED, val));
	if (ret)
		return ret;

	/* PLL - LPF */
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_2,
				 AN8855_RG_DA_QP_PLL_RICO_SEL_INTF |
				 AN8855_RG_DA_QP_PLL_FBKSEL_INTF |
				 AN8855_RG_DA_QP_PLL_BR_INTF |
				 AN8855_RG_DA_QP_PLL_BPD_INTF |
				 AN8855_RG_DA_QP_PLL_BPA_INTF |
				 AN8855_RG_DA_QP_PLL_BC_INTF,
				 AN8855_RG_DA_QP_PLL_RICO_SEL_INTF |
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_FBKSEL_INTF, 0x0) |
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_BR_INTF, 0x3) |
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_BPD_INTF, 0x0) |
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_BPA_INTF, 0x5) |
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_BC_INTF, 0x1));
	if (ret)
		return ret;

	/* PLL - ICO */
	ret = regmap_set_bits(priv->regmap, AN8855_PLL_CTRL_4,
			      AN8855_RG_DA_QP_PLL_ICOLP_EN_INTF);
	if (ret)
		return ret;
	ret = regmap_clear_bits(priv->regmap, AN8855_PLL_CTRL_2,
				AN8855_RG_DA_QP_PLL_ICOIQ_EN_INTF);
	if (ret)
		return ret;

	/* PLL - CHP */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x6;
	else
		val = 0x4;
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_2,
				 AN8855_RG_DA_QP_PLL_IR_INTF,
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_IR_INTF, val));
	if (ret)
		return ret;

	/* PLL - PFD */
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_2,
				 AN8855_RG_DA_QP_PLL_PFD_OFFSET_EN_INTRF |
				 AN8855_RG_DA_QP_PLL_PFD_OFFSET_INTF |
				 AN8855_RG_DA_QP_PLL_KBAND_PREDIV_INTF,
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_PFD_OFFSET_INTF, 0x1) |
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_KBAND_PREDIV_INTF, 0x1));
	if (ret)
		return ret;

	/* PLL - POSTDIV */
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_2,
				 AN8855_RG_DA_QP_PLL_POSTDIV_EN_INTF |
				 AN8855_RG_DA_QP_PLL_PHY_CK_EN_INTF |
				 AN8855_RG_DA_QP_PLL_PCK_SEL_INTF,
				 AN8855_RG_DA_QP_PLL_PCK_SEL_INTF);
	if (ret)
		return ret;

	/* PLL - SDM */
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_2,
				 AN8855_RG_DA_QP_PLL_SDM_HREN_INTF,
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_SDM_HREN_INTF, 0x0));
	if (ret)
		return ret;
	ret = regmap_clear_bits(priv->regmap, AN8855_PLL_CTRL_2,
				AN8855_RG_DA_QP_PLL_SDM_IFM_INTF);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_SS_LCPLL_PWCTL_SETTING_2,
				 AN8855_RG_NCPO_ANA_MSB,
				 FIELD_PREP(AN8855_RG_NCPO_ANA_MSB, 0x1));
	if (ret)
		return ret;

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x7a000000;
	else
		val = 0x48000000;
	ret = regmap_write(priv->regmap, AN8855_SS_LCPLL_TDC_FLT_2,
			   FIELD_PREP(AN8855_RG_LCPLL_NCPO_VALUE, val));
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, AN8855_SS_LCPLL_TDC_PCW_1,
			   FIELD_PREP(AN8855_RG_LCPLL_PON_HRDDS_PCW_NCPO_GPON, val));
	if (ret)
		return ret;

	ret = regmap_clear_bits(priv->regmap, AN8855_SS_LCPLL_TDC_FLT_5,
				AN8855_RG_LCPLL_NCPO_CHG);
	if (ret)
		return ret;
	ret = regmap_clear_bits(priv->regmap, AN8855_PLL_CK_CTRL_0,
				AN8855_RG_DA_QP_PLL_SDM_DI_EN_INTF);
	if (ret)
		return ret;

	/* PLL - SS */
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_3,
				 AN8855_RG_DA_QP_PLL_SSC_DELTA_INTF,
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_SSC_DELTA_INTF, 0x0));
	if (ret)
		return ret;
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_4,
				 AN8855_RG_DA_QP_PLL_SSC_DIR_DLY_INTF,
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_SSC_DIR_DLY_INTF, 0x0));
	if (ret)
		return ret;
	ret = regmap_update_bits(priv->regmap, AN8855_PLL_CTRL_3,
				 AN8855_RG_DA_QP_PLL_SSC_PERIOD_INTF,
				 FIELD_PREP(AN8855_RG_DA_QP_PLL_SSC_PERIOD_INTF, 0x0));
	if (ret)
		return ret;

	/* PLL - TDC */
	ret = regmap_clear_bits(priv->regmap, AN8855_PLL_CK_CTRL_0,
				AN8855_RG_DA_QP_PLL_TDC_TXCK_SEL_INTF);
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, AN8855_RG_QP_PLL_SDM_ORD,
			      AN8855_RG_QP_PLL_SSC_TRI_EN);
	if (ret)
		return ret;
	ret = regmap_set_bits(priv->regmap, AN8855_RG_QP_PLL_SDM_ORD,
			      AN8855_RG_QP_PLL_SSC_PHASE_INI);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_RX_DAC_EN,
				 AN8855_RG_QP_SIGDET_HF,
				 FIELD_PREP(AN8855_RG_QP_SIGDET_HF, 0x2));
	if (ret)
		return ret;

	/* TCL Disable (only for Co-SIM) */
	ret = regmap_clear_bits(priv->regmap, AN8855_PON_RXFEDIG_CTRL_0,
				AN8855_RG_QP_EQ_RX500M_CK_SEL);
	if (ret)
		return ret;

	/* TX Init */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x4;
	else
		val = 0x0;
	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_TX_MODE,
				 AN8855_RG_QP_TX_RESERVE |
				 AN8855_RG_QP_TX_MODE_16B_EN,
				 FIELD_PREP(AN8855_RG_QP_TX_RESERVE, val));
	if (ret)
		return ret;

	/* RX Control/Init */
	ret = regmap_set_bits(priv->regmap, AN8855_RG_QP_RXAFE_RESERVE,
			      AN8855_RG_QP_CDR_PD_10B_EN);
	if (ret)
		return ret;

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x1;
	else
		val = 0x2;
	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_CDR_LPF_MJV_LIM,
				 AN8855_RG_QP_CDR_LPF_RATIO,
				 FIELD_PREP(AN8855_RG_QP_CDR_LPF_RATIO, val));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_CDR_LPF_SETVALUE,
				 AN8855_RG_QP_CDR_PR_BUF_IN_SR |
				 AN8855_RG_QP_CDR_PR_BETA_SEL,
				 FIELD_PREP(AN8855_RG_QP_CDR_PR_BUF_IN_SR, 0x6) |
				 FIELD_PREP(AN8855_RG_QP_CDR_PR_BETA_SEL, 0x1));
	if (ret)
		return ret;

	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0xf;
	else
		val = 0xc;
	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_CDR_PR_CKREF_DIV1,
				 AN8855_RG_QP_CDR_PR_DAC_BAND,
				 FIELD_PREP(AN8855_RG_QP_CDR_PR_DAC_BAND, val));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_CDR_PR_KBAND_DIV_PCIE,
				 AN8855_RG_QP_CDR_PR_KBAND_PCIE_MODE |
				 AN8855_RG_QP_CDR_PR_KBAND_DIV_PCIE_MASK,
				 FIELD_PREP(AN8855_RG_QP_CDR_PR_KBAND_DIV_PCIE_MASK, 0x19));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_CDR_FORCE_IBANDLPF_R_OFF,
				 AN8855_RG_QP_CDR_PHYCK_SEL |
				 AN8855_RG_QP_CDR_PHYCK_RSTB |
				 AN8855_RG_QP_CDR_PHYCK_DIV,
				 FIELD_PREP(AN8855_RG_QP_CDR_PHYCK_SEL, 0x2) |
				 FIELD_PREP(AN8855_RG_QP_CDR_PHYCK_DIV, 0x21));
	if (ret)
		return ret;

	ret = regmap_clear_bits(priv->regmap, AN8855_RG_QP_CDR_PR_KBAND_DIV_PCIE,
				AN8855_RG_QP_CDR_PR_XFICK_EN);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RG_QP_CDR_PR_CKREF_DIV1,
				 AN8855_RG_QP_CDR_PR_KBAND_DIV,
				 FIELD_PREP(AN8855_RG_QP_CDR_PR_KBAND_DIV, 0x4));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RX_CTRL_26,
				 AN8855_RG_QP_EQ_RETRAIN_ONLY_EN |
				 AN8855_RG_LINK_NE_EN |
				 AN8855_RG_LINK_ERRO_EN,
				 AN8855_RG_QP_EQ_RETRAIN_ONLY_EN |
				 AN8855_RG_LINK_ERRO_EN);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RX_DLY_0,
				 AN8855_RG_QP_RX_SAOSC_EN_H_DLY |
				 AN8855_RG_QP_RX_PI_CAL_EN_H_DLY,
				 FIELD_PREP(AN8855_RG_QP_RX_SAOSC_EN_H_DLY, 0x3f) |
				 FIELD_PREP(AN8855_RG_QP_RX_PI_CAL_EN_H_DLY, 0x6f));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RX_CTRL_42,
				 AN8855_RG_QP_EQ_EN_DLY,
				 FIELD_PREP(AN8855_RG_QP_EQ_EN_DLY, 0x150));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RX_CTRL_2,
				 AN8855_RG_QP_RX_EQ_EN_H_DLY,
				 FIELD_PREP(AN8855_RG_QP_RX_EQ_EN_H_DLY, 0x150));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_PON_RXFEDIG_CTRL_9,
				 AN8855_RG_QP_EQ_LEQOSC_DLYCNT,
				 FIELD_PREP(AN8855_RG_QP_EQ_LEQOSC_DLYCNT, 0x1));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RX_CTRL_8,
				 AN8855_RG_DA_QP_SAOSC_DONE_TIME |
				 AN8855_RG_DA_QP_LEQOS_EN_TIME,
				 FIELD_PREP(AN8855_RG_DA_QP_SAOSC_DONE_TIME, 0x200) |
				 FIELD_PREP(AN8855_RG_DA_QP_LEQOS_EN_TIME, 0xfff));
	if (ret)
		return ret;

	/* Frequency meter */
	if (interface == PHY_INTERFACE_MODE_2500BASEX)
		val = 0x10;
	else
		val = 0x28;
	ret = regmap_update_bits(priv->regmap, AN8855_RX_CTRL_5,
				 AN8855_RG_FREDET_CHK_CYCLE,
				 FIELD_PREP(AN8855_RG_FREDET_CHK_CYCLE, val));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RX_CTRL_6,
				 AN8855_RG_FREDET_GOLDEN_CYCLE,
				 FIELD_PREP(AN8855_RG_FREDET_GOLDEN_CYCLE, 0x64));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, AN8855_RX_CTRL_7,
				 AN8855_RG_FREDET_TOLERATE_CYCLE,
				 FIELD_PREP(AN8855_RG_FREDET_TOLERATE_CYCLE, 0x2710));
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->regmap, AN8855_PLL_CTRL_0,
			      AN8855_RG_PHYA_AUTO_INIT);
	if (ret)
		return ret;

	/* PCS Init */
	if (interface == PHY_INTERFACE_MODE_SGMII &&
	    neg_mode == PHYLINK_PCS_NEG_INBAND_DISABLED) {
		ret = regmap_clear_bits(priv->regmap, AN8855_QP_DIG_MODE_CTRL_0,
					AN8855_RG_SGMII_MODE | AN8855_RG_SGMII_AN_EN);
		if (ret)
			return ret;
	}

	ret = regmap_clear_bits(priv->regmap, AN8855_RG_HSGMII_PCS_CTROL_1,
				AN8855_RG_TBI_10B_MODE);
	if (ret)
		return ret;

	if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED) {
		/* Set AN Ability - Interrupt */
		ret = regmap_set_bits(priv->regmap, AN8855_SGMII_REG_AN_FORCE_CL37,
				      AN8855_RG_FORCE_AN_DONE);
		if (ret)
			return ret;

		ret = regmap_update_bits(priv->regmap, AN8855_SGMII_REG_AN_13,
					 AN8855_SGMII_REMOTE_FAULT_DIS |
					 AN8855_SGMII_IF_MODE,
					 AN8855_SGMII_REMOTE_FAULT_DIS |
					 FIELD_PREP(AN8855_SGMII_IF_MODE, 0xb));
		if (ret)
			return ret;
	}

	/* Rate Adaption - GMII path config. */
	if (interface == PHY_INTERFACE_MODE_2500BASEX) {
		ret = regmap_clear_bits(priv->regmap, AN8855_RATE_ADP_P0_CTRL_0,
					AN8855_RG_P0_DIS_MII_MODE);
		if (ret)
			return ret;
	} else {
		if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED) {
			ret = regmap_set_bits(priv->regmap, AN8855_MII_RA_AN_ENABLE,
					      AN8855_RG_P0_RA_AN_EN);
			if (ret)
				return ret;
		} else {
			ret = regmap_update_bits(priv->regmap, AN8855_RG_AN_SGMII_MODE_FORCE,
						 AN8855_RG_FORCE_CUR_SGMII_MODE |
						 AN8855_RG_FORCE_CUR_SGMII_SEL,
						 AN8855_RG_FORCE_CUR_SGMII_SEL);
			if (ret)
				return ret;

			ret = regmap_clear_bits(priv->regmap, AN8855_RATE_ADP_P0_CTRL_0,
						AN8855_RG_P0_MII_RA_RX_EN |
						AN8855_RG_P0_MII_RA_TX_EN |
						AN8855_RG_P0_MII_RA_RX_MODE |
						AN8855_RG_P0_MII_RA_TX_MODE);
			if (ret)
				return ret;
		}

		ret = regmap_set_bits(priv->regmap, AN8855_RATE_ADP_P0_CTRL_0,
				      AN8855_RG_P0_MII_MODE);
		if (ret)
			return ret;
	}

	ret = regmap_set_bits(priv->regmap, AN8855_RG_RATE_ADAPT_CTRL_0,
			      AN8855_RG_RATE_ADAPT_RX_BYPASS |
			      AN8855_RG_RATE_ADAPT_TX_BYPASS |
			      AN8855_RG_RATE_ADAPT_RX_EN |
			      AN8855_RG_RATE_ADAPT_TX_EN);
	if (ret)
		return ret;

	/* Disable AN if not in autoneg */
	ret = regmap_update_bits(priv->regmap, AN8855_SGMII_REG_AN0, BMCR_ANENABLE,
				 neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED ? BMCR_ANENABLE :
									      0);

	if (interface == PHY_INTERFACE_MODE_SGMII &&
	    neg_mode == PHYLINK_PCS_NEG_INBAND_DISABLED) {
		ret = regmap_set_bits(priv->regmap, AN8855_PHY_RX_FORCE_CTRL_0,
				      AN8855_RG_FORCE_TXC_SEL);
		if (ret)
			return ret;
	}

	/* Force Speed with fixed-link or 2500base-x as doesn't support aneg */
	if (interface == PHY_INTERFACE_MODE_2500BASEX ||
	    neg_mode != PHYLINK_PCS_NEG_INBAND_ENABLED) {
		if (interface == PHY_INTERFACE_MODE_2500BASEX)
			val = AN8855_RG_LINK_MODE_P0_SPEED_2500;
		else
			val = AN8855_RG_LINK_MODE_P0_SPEED_1000;
		ret = regmap_update_bits(priv->regmap, AN8855_SGMII_STS_CTRL_0,
					 AN8855_RG_LINK_MODE_P0 |
					 AN8855_RG_FORCE_SPD_MODE_P0,
					 val | AN8855_RG_FORCE_SPD_MODE_P0);
		if (ret)
			return ret;
	}

	/* bypass flow control to MAC */
	ret = regmap_write(priv->regmap, AN8855_MSG_RX_LIK_STS_0,
			   AN8855_RG_DPX_STS_P3 | AN8855_RG_DPX_STS_P2 |
			   AN8855_RG_DPX_STS_P1 | AN8855_RG_TXFC_STS_P0 |
			   AN8855_RG_RXFC_STS_P0 | AN8855_RG_DPX_STS_P0);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, AN8855_MSG_RX_LIK_STS_2,
			   AN8855_RG_RXFC_AN_BYPASS_P3 |
			   AN8855_RG_RXFC_AN_BYPASS_P2 |
			   AN8855_RG_RXFC_AN_BYPASS_P1 |
			   AN8855_RG_TXFC_AN_BYPASS_P3 |
			   AN8855_RG_TXFC_AN_BYPASS_P2 |
			   AN8855_RG_TXFC_AN_BYPASS_P1 |
			   AN8855_RG_DPX_AN_BYPASS_P3 |
			   AN8855_RG_DPX_AN_BYPASS_P2 |
			   AN8855_RG_DPX_AN_BYPASS_P1 |
			   AN8855_RG_DPX_AN_BYPASS_P0);
	if (ret)
		return ret;

	return 0;
}

static void an8855_pcs_an_restart(struct phylink_pcs *pcs)
{
	struct an8855_priv *priv = container_of(pcs, struct an8855_priv, pcs);

	regmap_set_bits(priv->regmap, AN8855_SGMII_REG_AN0, BMCR_ANRESTART);
}

static const struct phylink_pcs_ops an8855_pcs_ops = {
	.pcs_get_state = an8855_pcs_get_state,
	.pcs_config = an8855_pcs_config,
	.pcs_an_restart = an8855_pcs_an_restart,
};

static const struct phylink_mac_ops an8855_phylink_mac_ops = {
	.mac_select_pcs	= an8855_phylink_mac_select_pcs,
	.mac_config	= an8855_phylink_mac_config,
	.mac_link_down	= an8855_phylink_mac_link_down,
	.mac_link_up	= an8855_phylink_mac_link_up,
};

static const struct dsa_switch_ops an8855_switch_ops = {
	.get_tag_protocol = an8855_get_tag_protocol,
	.setup = an8855_setup,
	.get_strings = an8855_get_strings,
	.get_ethtool_stats = an8855_get_ethtool_stats,
	.get_sset_count = an8855_get_sset_count,
	.port_enable = an8855_port_enable,
	.port_disable = an8855_port_disable,
	.port_stp_state_set = an8855_stp_state_set,
	.port_bridge_join = an8855_port_bridge_join,
	.port_bridge_leave = an8855_port_bridge_leave,
	.port_fdb_add = an8855_port_fdb_add,
	.port_fdb_del = an8855_port_fdb_del,
	.port_fdb_dump = an8855_port_fdb_dump,
	.port_vlan_filtering = an8855_port_vlan_filtering,
	.port_vlan_add = an8855_port_vlan_add,
	.port_vlan_del = an8855_port_vlan_del,
	.port_mirror_add = an8855_port_mirror_add,
	.port_mirror_del = an8855_port_mirror_del,
	.phylink_get_caps = an8855_phylink_get_caps,
	.get_phy_flags = en8855_get_phy_flags,
	.get_mac_eee = an8855_get_mac_eee,
	.set_mac_eee = an8855_set_mac_eee,
};

static int an8855_read_switch_id(struct an8855_priv *priv)
{
	u32 id;
	int ret;

	ret = regmap_read(priv->regmap, AN8855_CREV, &id);
	if (ret)
		return ret;

	if (id != AN8855_ID) {
		dev_err(priv->dev,
			"Switch id detected %x but expected %x",
			id, AN8855_ID);
		return -ENODEV;
	}

	return 0;
}

static int an8855_efuse_read(void *context, unsigned int offset,
			     void *val, size_t bytes)
{
	struct an8855_priv *priv = context;

	return regmap_bulk_read(priv->regmap, AN8855_EFUSE_DATA0 + offset,
				val, bytes / sizeof(u32));
}

static struct nvmem_config an8855_nvmem_config = {
	.name = "an8855-efuse",
	.size = AN8855_EFUSE_CELL * sizeof(u32),
	.stride = sizeof(u32),
	.word_size = sizeof(u32),
	.reg_read = an8855_efuse_read,
};

static int an8855_sw_register_nvmem(struct an8855_priv *priv)
{
	struct nvmem_device *nvmem;

	an8855_nvmem_config.priv = priv;
	an8855_nvmem_config.dev = priv->dev;
	nvmem = devm_nvmem_register(priv->dev, &an8855_nvmem_config);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	return 0;
}

static int
an8855_sw_probe(struct mdio_device *mdiodev)
{
	struct an8855_priv *priv;
	u32 val;
	int ret;

	priv = devm_kzalloc(&mdiodev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->bus = mdiodev->bus;
	priv->dev = &mdiodev->dev;
	priv->phy_base = mdiodev->addr;
	priv->phy_require_calib = of_property_read_bool(priv->dev->of_node,
							"airoha,ext-surge");

	priv->reset_gpio = devm_gpiod_get_optional(priv->dev, "reset",
						   GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	priv->regmap = devm_regmap_init(&mdiodev->dev, NULL, priv,
					&an8855_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(priv->dev, "regmap initialization failed");
		return PTR_ERR(priv->regmap);
	}

	if (priv->reset_gpio) {
		usleep_range(100000, 150000);
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		usleep_range(100000, 150000);
		gpiod_set_value_cansleep(priv->reset_gpio, 1);

		/* Poll HWTRAP reg to wait for Switch to fully Init */
		ret = regmap_read_poll_timeout(priv->regmap, AN8855_HWTRAP, val,
					       val, 20, 200000);
		if (ret)
			return ret;
	}

	ret = an8855_read_switch_id(priv);
	if (ret)
		return ret;

	priv->ds = devm_kzalloc(&mdiodev->dev, sizeof(*priv->ds), GFP_KERNEL);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->dev = &mdiodev->dev;
	priv->ds->num_ports = AN8855_NUM_PORTS;
	priv->ds->priv = priv;
	priv->ds->ops = &an8855_switch_ops;
	mutex_init(&priv->reg_mutex);
	priv->ds->phylink_mac_ops = &an8855_phylink_mac_ops;

	priv->pcs.ops = &an8855_pcs_ops;
	priv->pcs.neg_mode = true;
	priv->pcs.poll = true;

	ret = an8855_sw_register_nvmem(priv);
	if (ret)
		return ret;

	dev_set_drvdata(&mdiodev->dev, priv);

	return dsa_register_switch(priv->ds);
}

static void
an8855_sw_remove(struct mdio_device *mdiodev)
{
	struct an8855_priv *priv = dev_get_drvdata(&mdiodev->dev);

	dsa_unregister_switch(priv->ds);
	mutex_destroy(&priv->reg_mutex);
}

static const struct of_device_id an8855_of_match[] = {
	{ .compatible = "airoha,an8855" },
	{ /* sentinel */ },
};

static struct mdio_driver an8855_mdio_driver = {
	.probe = an8855_sw_probe,
	.remove = an8855_sw_remove,
	.mdiodrv.driver = {
		.name = "an8855",
		.of_match_table = an8855_of_match,
	},
};

mdio_module_driver(an8855_mdio_driver);

MODULE_AUTHOR("Min Yao <min.yao@airoha.com>");
MODULE_AUTHOR("Christian Marangi <ansuelsmth@gmail.com>");
MODULE_DESCRIPTION("Driver for Airoha AN8855 Switch");
MODULE_LICENSE("GPL");
