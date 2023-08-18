#include <linux/if.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/stat.h>
#include <linux/mod_devicetable.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/skbuff.h>
#include <linux/netdev_features.h>
#include <linux/spinlock.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/of_net.h>
#include <linux/clk.h>
#include <linux/mii.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/dmaengine.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/debugfs.h>
#include <crypto/hash.h>

#define DMA_DESC_RX 256
#define DMA_DESC_TX 256
#define BUDGET		(dma_desc_rx / 4)

#define POWER_CHAN_NUM 3

#define SUNXI_GMAC_BASIC_CTL0 0x00
#define SUNXI_GMAC_BASIC_CTL1 0x4
#define SUNXI_GMAC_TX_CTL0  0x10
#define SUNXI_GMAC_TX_CTL1  0x14

#define SUNXI_GMAC_RX_CTL0  0x24
#define SUNXI_GMAC_RX_CTL1  0x28

#define SUNXI_GMAC_RGMII_STA 0x80
#define SUNXI_GMAC_RGMII_IRQ    0x01

#define SUNXI_GMAC_INT_STA 0x08

#define SUNXI_GMAC_HASH_TABLE_SIZE 64

#define SUNXI_GMAC_TX_UNF_INT 0x00010
#define SUNXI_GMAC_TX_TOUT_INT 0x00008
#define SUNXI_GMAC_RX_OVF_INT   0x01000
#define SUNXI_GMAC_RX_UA_INT    0x00200
#define SUNXI_GMAC_RX_STOP_INT  0x00400
#define SUNXI_GMAC_RX_TOUT_INT  0x00800
#define SUNXI_GMAC_TX_EARLY_INT 0x00020
#define SUNXI_GMAC_RX_EARLY_INT 0x02000
#define SUNXI_GMAC_TX_STOP_INT  0x00002

#define SUNXI_GMAC_TX_INT   0x00001
#define SUNXI_GMAC_TX_UA_INT    0x00004
#define SUNXI_GMAC_RX_INT   0x00100

#define SUNXI_GMAC_TX_THRESH (dma_desc_tx / 4)

#define SOFT_RST    1
#define SF_DMA_MODE 1

#define CTL0_DM 0x01

#define TX_MD   0x02
#define TX_NEXT_FRM 0x04
#define TX_TH   0x0700

#define RX_MD   0x02
#define RX_TH   0x0030
#define RX_RUNT_FRM 0x04
#define RX_ERR_FRM  0x08

#define SUNXI_GMAC_TX_DESC_LIST 0x20
#define SUNXI_GMAC_RX_DESC_LIST 0x34
#define SUNXI_GMAC_RX_FRM_FLT   0x38

#define RX_INT 0x00100
#define TX_INT 0x00001
#define TX_UNF_INT  0x0010

#define SUNXI_GMAC_INT_EN   0x0c
#define SUNXI_GMAC_TX_FLOW_CTL  0x1c

#define SUNXI_GMAC_ADDR_HI(_reg) (0x00 + ((_reg) << 3))
#define SUNXI_GMAC_ADDR_LO(_reg) (0x04 + ((_reg) << 3))

#define MAX_BUF_SZ  (SZ_2K - 1)

#define FLOW_RX 1
#define FLOW_TX 2

#define SUNXI_GMAC_FRAME_FILTER_PR  0x01
#define SUNXI_GMAC_FRAME_FILTER_PM  0x10
#define SUNXI_GMAC_FRAME_FILTER_HMC 0x04

#define SUNXI_GMAC_RX_HASH0 0x40
#define SUNXI_GMAC_RX_HASH1 0x44

#define circ_cnt(head, tail, size) (((head) > (tail)) ? \
					((head) - (tail)) : \
					((head) - (tail)) & ((size) - 1))

#define circ_space(head, tail, size) circ_cnt((tail), ((head) + 1), (size))

#define circ_inc(n, s) (((n) + 1) % (s))


#define TX_TIMEO    5000
static int watchdog = TX_TIMEO;
module_param(watchdog, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(watchdog, "Transmit timeout in milliseconds");

static int rxmode = 1;
module_param(rxmode, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rxmode, "DMA threshold control value");

static int txmode = 1;
module_param(txmode, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(txmode, "DMA threshold control value");

#define GETH_MAC_ADDRESS "00:00:00:00:00:00"
static char *mac_str = GETH_MAC_ADDRESS;
module_param(mac_str, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mac_str, "MAC Address String.(xx:xx:xx:xx:xx:xx)");

static int pause = 0x400;
module_param(pause, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");

static int dma_desc_rx = DMA_DESC_RX;
module_param(dma_desc_rx, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_desc_rx, "The number of receive's descriptors");

static int dma_desc_tx = DMA_DESC_TX;
module_param(dma_desc_tx, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_desc_tx, "The number of transmit's descriptors");

static int flow_ctrl = 0;
module_param(flow_ctrl, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(flow_ctrl, "Flow control [0: off, 1: rx, 2: tx, 3: both]");

static u64 sunxi_gmac_dma_mask = DMA_BIT_MASK(32);

enum tx_dma_irq_status {
	tx_hard_error = 1,
	tx_hard_error_bump_tc = 2,
	handle_tx_rx = 3,
};

struct sunxi_gmac_extra_stats {
	/* Transmit errors */
	unsigned long tx_underflow;
	unsigned long tx_carrier;
	unsigned long tx_losscarrier;
	unsigned long vlan_tag;
	unsigned long tx_deferred;
	unsigned long tx_vlan;
	unsigned long tx_jabber;
	unsigned long tx_frame_flushed;
	unsigned long tx_payload_error;
	unsigned long tx_ip_header_error;

	/* Receive errors */
	unsigned long rx_desc;
	unsigned long sa_filter_fail;
	unsigned long overflow_error;
	unsigned long ipc_csum_error;
	unsigned long rx_collision;
	unsigned long rx_crc;
	unsigned long dribbling_bit;
	unsigned long rx_length;
	unsigned long rx_mii;
	unsigned long rx_multicast;
	unsigned long rx_gmac_overflow;
	unsigned long rx_watchdog;
	unsigned long da_rx_filter_fail;
	unsigned long sa_rx_filter_fail;
	unsigned long rx_missed_cntr;
	unsigned long rx_overflow_cntr;
	unsigned long rx_vlan;

	/* Tx/Rx IRQ errors */
	unsigned long tx_undeflow_irq;
	unsigned long tx_process_stopped_irq;
	unsigned long tx_jabber_irq;
	unsigned long rx_overflow_irq;
	unsigned long rx_buf_unav_irq;
	unsigned long rx_process_stopped_irq;
	unsigned long rx_watchdog_irq;
	unsigned long tx_early_irq;
	unsigned long fatal_bus_error_irq;

	/* Extra info */
	unsigned long threshold;
	unsigned long tx_pkt_n;
	unsigned long rx_pkt_n;
	unsigned long poll_n;
	unsigned long sched_timer_n;
	unsigned long normal_irq_n;
};

typedef union {
	struct {
		/* TDES0 */
		unsigned int deferred:1;	/* Deferred bit (only half-duplex) */
		unsigned int under_err:1;	/* Underflow error */
		unsigned int ex_deferral:1;	/* Excessive deferral */
		unsigned int coll_cnt:4;	/* Collision count */
		unsigned int vlan_tag:1;	/* VLAN Frame */
		unsigned int ex_coll:1;		/* Excessive collision */
		unsigned int late_coll:1;	/* Late collision */
		unsigned int no_carr:1;		/* No carrier */
		unsigned int loss_carr:1;	/* Loss of collision */
		unsigned int ipdat_err:1;	/* IP payload error */
		unsigned int frm_flu:1;		/* Frame flushed */
		unsigned int jab_timeout:1;	/* Jabber timeout */
		unsigned int err_sum:1;		/* Error summary */
		unsigned int iphead_err:1;	/* IP header error */
		unsigned int ttss:1;		/* Transmit time stamp status */
		unsigned int reserved0:13;
		unsigned int own:1;		/* Own bit. CPU:0, DMA:1 */
	} tx;

	/* bits 5 7 0 | Frame status
	 * ----------------------------------------------------------
	 *      0 0 0 | IEEE 802.3 Type frame (length < 1536 octects)
	 *      1 0 0 | IPv4/6 No CSUM errorS.
	 *      1 0 1 | IPv4/6 CSUM PAYLOAD error
	 *      1 1 0 | IPv4/6 CSUM IP HR error
	 *      1 1 1 | IPv4/6 IP PAYLOAD AND HEADER errorS
	 *      0 0 1 | IPv4/6 unsupported IP PAYLOAD
	 *      0 1 1 | COE bypassed.. no IPv4/6 frame
	 *      0 1 0 | Reserved.
	 */
	struct {
		/* RDES0 */
		unsigned int chsum_err:1;	/* Payload checksum error */
		unsigned int crc_err:1;		/* CRC error */
		unsigned int dribbling:1;	/* Dribble bit error */
		unsigned int mii_err:1;		/* Received error (bit3) */
		unsigned int recv_wt:1;		/* Received watchdog timeout */
		unsigned int frm_type:1;	/* Frame type */
		unsigned int late_coll:1;	/* Late Collision */
		unsigned int ipch_err:1;	/* IPv header checksum error (bit7) */
		unsigned int last_desc:1;	/* Laset descriptor */
		unsigned int first_desc:1;	/* First descriptor */
		unsigned int vlan_tag:1;	/* VLAN Tag */
		unsigned int over_err:1;	/* Overflow error (bit11) */
		unsigned int len_err:1;		/* Length error */
		unsigned int sou_filter:1;	/* Source address filter fail */
		unsigned int desc_err:1;	/* Descriptor error */
		unsigned int err_sum:1;		/* Error summary (bit15) */
		unsigned int frm_len:14;	/* Frame length */
		unsigned int des_filter:1;	/* Destination address filter fail */
		unsigned int own:1;		/* Own bit. CPU:0, DMA:1 */
	#define RX_PKT_OK		0x7FFFB77C
	#define RX_LEN			0x3FFF0000
	} rx;

	unsigned int all;
} desc0_u;

typedef union {
	struct {
		/* TDES1 */
		unsigned int buf1_size:11;	/* Transmit buffer1 size */
		unsigned int buf2_size:11;	/* Transmit buffer2 size */
		unsigned int ttse:1;		/* Transmit time stamp enable */
		unsigned int dis_pad:1;		/* Disable pad (bit23) */
		unsigned int adr_chain:1;	/* Second address chained */
		unsigned int end_ring:1;	/* Transmit end of ring */
		unsigned int crc_dis:1;		/* Disable CRC */
		unsigned int cic:2;		/* Checksum insertion control (bit27:28) */
		unsigned int first_sg:1;	/* First Segment */
		unsigned int last_seg:1;	/* Last Segment */
		unsigned int interrupt:1;	/* Interrupt on completion */
	} tx;

	struct {
		/* RDES1 */
		unsigned int buf1_size:11;	/* Received buffer1 size */
		unsigned int buf2_size:11;	/* Received buffer2 size */
		unsigned int reserved1:2;
		unsigned int adr_chain:1;	/* Second address chained */
		unsigned int end_ring:1;		/* Received end of ring */
		unsigned int reserved2:5;
		unsigned int dis_ic:1;		/* Disable interrupt on completion */
	} rx;

	unsigned int all;
} desc1_u;

typedef struct dma_desc {
	desc0_u desc0;
	desc1_u desc1;
	/* The address of buffers */
	unsigned int	desc2;
	/* Next desc's address */
	unsigned int	desc3;
} __attribute__((packed)) dma_desc_t;

enum rx_frame_status { /* IPC status */
	good_frame = 0,
	discard_frame = 1,
	csum_none = 2,
	llc_snap = 4,
};

struct sunxi_gmac_priv {
    struct net_device *netdev;
    void __iomem *mem[3];
    struct device *dev;
    phy_interface_t mode;
    struct napi_struct napi;
    spinlock_t lock;
#define EXT_PHY 1
#define INT_PHY 0

#define CLK_GMAC 0
#define CLK_EPHY 1
    int phy_ext;
    struct resource *res[3];
    struct clk *clk[2];
    struct regulator *power[3];
    phy_interface_t phy_interface;
    u32 tx_delay;
    u32 rx_delay;
    // int reset;
    // int rst_active_low;
    struct pinctrl *pinctrl;
    struct device_node *np;
    struct phy_device *phydev;
    struct sk_buff **rx_skb;
    struct sk_buff **tx_skb;
    bool is_suspend;
    struct dma_desc *tx_dma;
    struct dma_desc *rx_dma;
    dma_addr_t dma_tx_phy;
    dma_addr_t dma_rx_phy;
    int buf_sz;
    u32 rx_clean;
    u32 rx_dirty;
    u32 tx_clean;
    u32 tx_dirty;

    struct sunxi_gmac_extra_stats xstats;
    // spinlock_t tx_lock;

    int duplex;
    int speed;
    int link;
    struct dentry *debugfs_dir;
    struct dentry *debugfs_regs;
    struct dentry *debugfs_rgmii;
};

static const struct of_device_id sunxi_gmac_ids[] = {
    {
        .compatible = "allwinner,sunxi-gmac",
    },
    {}
};

MODULE_DEVICE_TABLE(of, sunxi_gmac_ids);

static void sunxi_gmac_free_tx_skb(struct sunxi_gmac_priv *priv);
static void sunxi_gmac_desc_init_chain(struct dma_desc *desc, unsigned long addr, size_t size);
static void sunxi_gmac_tx_complete(struct sunxi_gmac_priv *priv);

static void sunxi_gmac_start_tx(struct sunxi_gmac_priv *priv, unsigned long base)
{
    u32 value;

    /* write the base address of Tx descriptor lists into registers */

    iowrite32(base, priv->mem[0] + SUNXI_GMAC_TX_DESC_LIST);

    value = ioread32(priv->mem[0] + SUNXI_GMAC_TX_CTL1);
    value |= 0x40000000;
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_TX_CTL1);
}

static void sunxi_gmac_stop_tx(struct sunxi_gmac_priv *priv)
{
    u32 value = ioread32(priv->mem[0] + SUNXI_GMAC_TX_CTL1);

    value &= ~0x40000000;
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_TX_CTL1);
}

static void sunxi_gmac_start_rx(struct sunxi_gmac_priv *priv, unsigned long base)
{
    u32 value;

    /* Write the base address of Rx descriptor lists into registers */
    iowrite32(base, priv->mem[0] + SUNXI_GMAC_RX_DESC_LIST);

    value = ioread32(priv->mem[0] + SUNXI_GMAC_RX_CTL1);
    value |= 0x40000000;
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_RX_CTL1);
}

static void sunxi_gmac_stop_rx(struct sunxi_gmac_priv *priv)
{
    u32 value;

    value = ioread32(priv->mem[0] + SUNXI_GMAC_RX_CTL1);
    value &= ~0x40000000;
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_RX_CTL1);
}

static void sunxi_gmac_enable(struct sunxi_gmac_priv *priv)
{
    u32 value;

    value = ioread32(priv->mem[0] + SUNXI_GMAC_TX_CTL0);
    value |= (1 << 31);
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_TX_CTL0);

    value = ioread32(priv->mem[0] + SUNXI_GMAC_RX_CTL0);
    value |= (1 << 31);
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_RX_CTL0);
}

static void sunxi_gmac_disable(struct sunxi_gmac_priv *priv)
{
    u32 value;

    value = ioread32(priv->mem[0] + SUNXI_GMAC_TX_CTL0);
    value &= ~(1 << 31);
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_TX_CTL0);

    value = ioread32(priv->mem[0] + SUNXI_GMAC_RX_CTL0);
    value &= ~(1 << 31);
    iowrite32(value, priv->mem[0] + SUNXI_GMAC_RX_CTL0);
}

int desc_buf_get_len(struct dma_desc *desc)
{
	return (desc->desc1.all & ((1 << 11) - 1));
}

int desc_buf_get_addr(struct dma_desc *desc)
{
	return desc->desc2;
}

static int sunxi_gmac_int_status(struct sunxi_gmac_priv *priv, struct sunxi_gmac_extra_stats *stats)
{
    int ret = 0;
    u32 intr;

    intr = ioread32(priv->mem[1] + SUNXI_GMAC_RGMII_STA);
    if (intr & SUNXI_GMAC_RGMII_IRQ) {
        ioread32(priv->mem[1] + SUNXI_GMAC_RGMII_STA);
    }

    intr = ioread32(priv->mem[0] + SUNXI_GMAC_INT_STA);

    if (intr & SUNXI_GMAC_TX_UNF_INT) {
        ret = tx_hard_error_bump_tc;
        stats->tx_undeflow_irq++;
    }

    if (intr & SUNXI_GMAC_TX_TOUT_INT) {
        stats->tx_jabber_irq++;
    }

    if (intr & SUNXI_GMAC_RX_OVF_INT) {
        stats->rx_overflow_irq++;
    }

    if (intr & SUNXI_GMAC_RX_UA_INT) {
        stats->rx_buf_unav_irq++;
    }

    if (intr & SUNXI_GMAC_RX_STOP_INT) {
        stats->rx_process_stopped_irq++;
    }

    if (intr & SUNXI_GMAC_RX_TOUT_INT) {
        stats->rx_watchdog_irq++;
    }

    if (intr & SUNXI_GMAC_TX_STOP_INT) {
        stats->tx_process_stopped_irq++;
        ret = tx_hard_error;
    }

    if (intr & 
        (SUNXI_GMAC_TX_INT | 
         SUNXI_GMAC_RX_INT | 
         SUNXI_GMAC_RX_EARLY_INT | 
         SUNXI_GMAC_TX_UA_INT)) {
        stats->normal_irq_n++;
        if (intr & (SUNXI_GMAC_TX_INT | SUNXI_GMAC_RX_INT)) {
            ret = handle_tx_rx;
        }
    }

    iowrite32(intr & 0x3fff, priv->mem[0] + SUNXI_GMAC_INT_STA);

    return ret;
}

static void sunxi_gmac_int_disable(struct sunxi_gmac_priv *priv)
{
    iowrite32(0, priv->mem[0] + SUNXI_GMAC_INT_EN);
}

static void sunxi_gmac_int_enable(struct sunxi_gmac_priv *priv)
{
    iowrite32(SUNXI_GMAC_RX_INT |
              SUNXI_GMAC_TX_INT |
              SUNXI_GMAC_TX_UNF_INT, 
            priv->mem[0] + SUNXI_GMAC_INT_EN);
}

static void sunxi_gmac_schedule(struct sunxi_gmac_priv *priv)
{
    if (likely(napi_schedule_prep(&priv->napi))) {
        sunxi_gmac_int_disable(priv);
        __napi_schedule(&priv->napi);
    }
}

static void sunxi_gmac_tx_err(struct sunxi_gmac_priv *priv)
{
    netif_stop_queue(priv->netdev);
    
    sunxi_gmac_stop_tx(priv);

    sunxi_gmac_free_tx_skb(priv);
    memset(priv->tx_dma, 0, dma_desc_tx * sizeof(struct dma_desc));

    sunxi_gmac_desc_init_chain(priv->tx_dma, (unsigned long)priv->dma_tx_phy, dma_desc_tx);
    priv->tx_dirty = 0;
    priv->tx_clean = 0;

    sunxi_gmac_start_tx(priv, priv->dma_tx_phy);

    priv->netdev->stats.tx_errors++;
    netif_wake_queue(priv->netdev);
}

static irqreturn_t sunxi_gmac_interrupt(int irq, void *dev_id)
{
    struct net_device *dev = (struct net_device *)dev_id;
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    int status;

    if (unlikely(!dev)) {
        return IRQ_NONE;
    }

    status = sunxi_gmac_int_status(priv, &priv->xstats);

    if (likely(status == handle_tx_rx)) {
        sunxi_gmac_schedule(priv);
    } else if (unlikely(status == tx_hard_error_bump_tc)) {
        netdev_info(dev, "Do nothing for bump tc\n");
    } else if (unlikely(status == tx_hard_error)) {
        sunxi_gmac_tx_err(priv);
    } else {
        netdev_info(dev, "Do nothing ...\n");
    }

    return IRQ_HANDLED;
}

static void sunxi_gmac_flow_ctrl(struct sunxi_gmac_priv *priv, int duplex, int fc, int pause)
{
    u32 flow = 0;

    if (fc & FLOW_RX) {
        flow = ioread32(priv->mem[0] + SUNXI_GMAC_RX_CTL0);
        flow |= 0x10000;
        iowrite32(flow, priv->mem[0] + SUNXI_GMAC_RX_CTL0);
    }

    if (fc & FLOW_TX) {
        flow = ioread32(priv->mem[0] + SUNXI_GMAC_TX_FLOW_CTL);
        flow |= 0x00001;
        iowrite32(flow, priv->mem[0] + SUNXI_GMAC_TX_FLOW_CTL);
    }

    if (duplex) {
        flow = ioread32(priv->mem[0] + SUNXI_GMAC_TX_FLOW_CTL);
        flow |= (pause << 4);
        iowrite32(flow, priv->mem[0] + SUNXI_GMAC_TX_FLOW_CTL);
    }
}

static void sunxi_gmac_set_link_mode(struct sunxi_gmac_priv *priv, int duplex, int speed)
{
    u32 ctrl = ioread32(priv->mem[0] + SUNXI_GMAC_BASIC_CTL0);

    if (!duplex)
        ctrl &= ~CTL0_DM;
    else 
        ctrl |= CTL0_DM;

    switch(speed) {
        case 1000:
            ctrl &= ~0x0c;
            break;
        case 100:
        case 10:
        default:
            ctrl |= 0x08;
            if (speed == 100)
                ctrl |= 0x04;
            else
                ctrl &= ~0x04;
            break;
    }

    iowrite32(ctrl, priv->mem[0] + SUNXI_GMAC_BASIC_CTL0);
}

static void sunxi_gmac_adjust_link(struct net_device *dev)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    struct phy_device *phydev = dev->phydev;

    unsigned long flags;
    int new_state = 0;

    if (!phydev)
        return;

    spin_lock_irqsave(&priv->lock, flags);

    if (phydev->link) {
        if (phydev->duplex != priv->duplex) {
            new_state = 1;
            priv->duplex = phydev->duplex;
        }

        if (phydev->pause) {
            sunxi_gmac_flow_ctrl(priv, phydev->duplex, flow_ctrl, pause);
        }

        if (phydev->speed != priv->speed) {
            new_state = 1;
            priv->speed = phydev->speed;
        }

        if (priv->link == 0) {
            new_state = 1;
            priv->link = phydev->link;
        }

        if (new_state) {
            sunxi_gmac_set_link_mode(priv, priv->duplex, priv->speed);
        }

#ifdef LOOPBACK_DEBUG
        phydev->state = PHY_FORCING;
#endif
    } else if (priv->link != phydev->link) {
        new_state = 1;
        priv->link = 0;
        priv->speed = 0;
        priv->duplex = -1;
    }

    if (new_state)
        phy_print_status(phydev);

    spin_unlock_irqrestore(&priv->lock, flags);
}

static void sunxi_gmac_desc_set_own(struct dma_desc *desc)
{
    desc->desc0.all |= 0x80000000;
}

static int sunxi_gmac_desc_get_own(struct dma_desc *desc)
{
    return desc->desc0.all & 0x80000000;
}

static void sunxi_gmac_desc_tx_close(struct dma_desc *first, struct dma_desc *end, int csum_insert)
{
    struct dma_desc *desc = first;

    first->desc1.tx.first_sg = 1;
    end->desc1.tx.last_seg = 1;
    end->desc1.tx.interrupt = 1;

    if (csum_insert) {
        do {
            desc->desc1.tx.cic = 3;
            desc++;
        } while(desc <= end);
    }
}

static void sunxi_gmac_desc_init(struct dma_desc *desc)
{
    desc->desc1.all = 0;
    desc->desc2 = 0;

    desc->desc1.all |= (1 << 24);
}

static void sunxi_gmac_desc_buf_set(struct dma_desc *desc, unsigned long paddr, size_t size)
{
    desc->desc1.all &= (~((1 << 11) -1 ));
    desc->desc1.all |= (size & ((1 << 11) -1 ));
    desc->desc2 = paddr;
}

static int sunxi_gmac_reset(struct sunxi_gmac_priv *priv)
{
    u32 val;

    /* DMA SW Reset */
    val = ioread32(priv->mem[0] + SUNXI_GMAC_BASIC_CTL1);
    val |= SOFT_RST;

    iowrite32(val, priv->mem[0] + SUNXI_GMAC_BASIC_CTL1);

    ndelay(10000);

    return !!(ioread32(priv->mem[0] + SUNXI_GMAC_BASIC_CTL1) & SOFT_RST);
}

static void sunxi_gmac_dma_init(void __iomem *iobase)
{
    u32 val;

    val = (8 << 24);
#ifdef CONFIG_GMAC_DA
    val |= RX_TX_PRI;   /* Rx has priority over tx */
#endif

    iowrite32(val, iobase + SUNXI_GMAC_BASIC_CTL1);

    /* Mask interrupts by writing to CSR7 */
    iowrite32(RX_INT | TX_INT | TX_UNF_INT, iobase + SUNXI_GMAC_INT_EN);
}

static int sunxi_gmac_init(struct sunxi_gmac_priv *priv, int tx_mode, int rx_mode)
{
    u32 val;

    sunxi_gmac_dma_init(priv->mem[0]);

    /* Initialize the core component */
    val = ioread32(priv->mem[0] + SUNXI_GMAC_TX_CTL0);
    val |= (1 << 30); /* Jabber Disable */
    iowrite32(val, priv->mem[0] + SUNXI_GMAC_TX_CTL0);

    val = ioread32(priv->mem[0] + SUNXI_GMAC_RX_CTL0);
    val |= (1 << 27);   /* Enable CRC & IPv4 Header checksum */
    val |= (1 << 28);   /* Automatic Pad/CRC Stripping */
    val |= (1 << 29);   /* Jumbo Frame Enable */
    iowrite32(val, priv->mem[0] + SUNXI_GMAC_RX_CTL0);

    /* Set the Rx&Tx mode */
    val = ioread32(priv->mem[0] + SUNXI_GMAC_TX_CTL1);

    if (tx_mode == SF_DMA_MODE) {
        /* Transmit COE type 2 cannot be done int cut-through mode. */
        val |= TX_MD;
        /* Operating on second frame increase the performance 
         * especially when transmit store-and-forward is used.
         */
         val |= TX_NEXT_FRM;
    } else {
        val &= ~TX_MD;
        val &= ~TX_TH;

        /* Set the transmit threshold */
        if (tx_mode <= 64)
            val |= 0;
        else if (tx_mode <= 128)
            val |= 0x100;
        else if (tx_mode <= 192)
            val |= 0x200;
        else
            val |= 0x300;
            
    }

    iowrite32(val, priv->mem[0] + SUNXI_GMAC_TX_CTL1);

    val = ioread32(priv->mem[0] + SUNXI_GMAC_RX_CTL1);
    if (rx_mode == SF_DMA_MODE) {
        val |= RX_MD;
    } else {
        val &= ~RX_MD;
        val &= ~RX_TH;
        if (rx_mode <= 32)
            val |= 0x10;
        else if (rx_mode <= 64)
            val |= 0x00;
        else if (rx_mode <= 96)
            val |= 0x20;
        else
            val |= 0x30;
    }

    val |= (RX_ERR_FRM | RX_RUNT_FRM);

    iowrite32(val, priv->mem[0] + SUNXI_GMAC_RX_CTL1);

    return 0;
}

static void sunxi_gmac_set_umac(struct sunxi_gmac_priv *priv, unsigned char *addr, int idx)
{
    unsigned long data;

    data = (addr[5] << 8) | addr[4];
    iowrite32(data, priv->mem[1] + SUNXI_GMAC_ADDR_HI(idx));
    data = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
    iowrite32(data, priv->mem[1] + SUNXI_GMAC_ADDR_LO(idx));
}

/* sunxi_gmac_dma_desc_init - initialize the RX/TX descriptor list
 * @dev: net device structure
 * Descriptor: initialize the list for dma
 */

static int sunxi_gmac_dma_desc_init(struct net_device *dev)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    u32 buf_sz;

    priv->rx_skb = kzalloc(sizeof(struct sk_buff *) * dma_desc_rx, GFP_KERNEL);
    if (!priv->rx_skb)
        return -ENOMEM;

    priv->tx_skb = kzalloc(sizeof(struct sk_buff *) * dma_desc_tx, GFP_KERNEL);
    if (!priv->tx_skb)
        goto tx_skb_err;

    /* Set the size of buffer depend on the MTU & max buf size */
    buf_sz = MAX_BUF_SZ;

    priv->tx_dma = dma_alloc_coherent(priv->dev, 
                   dma_desc_tx * sizeof(struct dma_desc),
                   &priv->dma_tx_phy,
                   GFP_KERNEL);
    if (!priv->tx_dma)
        goto dma_tx_err;

    priv->rx_dma = dma_alloc_coherent(priv->dev,
                   dma_desc_tx * sizeof(struct dma_desc),
                   &priv->dma_rx_phy,
                   GFP_KERNEL);

    if (!priv->rx_dma)
        goto dma_rx_err;

    priv->buf_sz = buf_sz;

    return 0;
dma_rx_err:
    dma_free_coherent(priv->dev, dma_desc_tx * sizeof(struct dma_desc), priv->tx_dma, priv->dma_tx_phy);
dma_tx_err:
    kfree(priv->tx_skb);
tx_skb_err:
    kfree(priv->rx_skb);

    return -ENOMEM;
}

static void sunxi_gmac_desc_init_chain(struct dma_desc *desc, unsigned long addr, size_t size)
{
    /* In chained mode the desc3 points to the next element in the ring.
	 * The latest element has to point to the head.
	 */
     int i;
     struct dma_desc *p = desc;
     unsigned long dma_phy = addr;

     for (i = 0; i < (size - 1); i++) {
        dma_phy += sizeof(struct dma_desc);
        p->desc3 = (u32)dma_phy;
        /* Chain mode */
        p->desc1.all |= (1 << 24);
        p++;
     }
     p->desc1.all |= (1 << 24);
     p->desc3 = (u32)addr;
}

static void sunxi_gmac_rx_refill(struct net_device *dev)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    struct dma_desc *desc;
    struct sk_buff *skb = NULL;
    dma_addr_t paddr;

    while(circ_space(priv->rx_clean, priv->rx_dirty, dma_desc_rx) > 0) {
        int entry = priv->rx_clean;

        /* Find the dirty's desc and clean it */
        desc = priv->rx_dma + entry;

        if (priv->rx_skb[entry] == NULL) {
            skb = netdev_alloc_skb_ip_align(dev, priv->buf_sz);
            if (unlikely(skb == NULL))
                break;

            priv->rx_skb[entry] = skb;
            paddr = dma_map_single(priv->dev, skb->data,
                                    priv->buf_sz, DMA_FROM_DEVICE);
            sunxi_gmac_desc_buf_set(desc, paddr, priv->buf_sz);
        }

        /* sync memory */

        wmb();
        sunxi_gmac_desc_set_own(desc);
        priv->rx_clean = circ_inc(priv->rx_clean, dma_desc_rx);
    }
}

static int sunxi_gmac_open(struct net_device *dev)
{
    int ret;
    struct sunxi_gmac_priv *priv = netdev_priv(dev);

    ret = sunxi_gmac_power_on(priv);

    if (ret) {
        netdev_err(dev, "Power on failed\n");
	return ret;
    }

    ret = sunxi_gmac_clk_enable(priv);
    if (ret) {
	netdev_err(dev, "enable clk failed\n");
	goto power_off;
    }

    if (!of_phy_get_and_connect(dev, priv->np, &sunxi_gmac_adjust_link)) {
        netdev_err(dev, "No phy found\n");
	goto disble_clk;
    }

    ret = request_irq(dev->irq, sunxi_gmac_interrupt, IRQF_SHARED, dev_name(priv->dev), dev);

    if (ret) {
        goto out;
    }

    ret = sunxi_gmac_reset(priv);

    if (ret) {
        netdev_err(dev, "mac reset failed\n");
        ret = -1;
        goto irq_free;
    }

    sunxi_gmac_init(priv, txmode, rxmode);
    sunxi_gmac_set_umac(priv, dev->dev_addr, 0);

    if (!priv->is_suspend) {
        ret = sunxi_gmac_dma_desc_init(dev);
        if (ret) {
            ret = -EINVAL;
            goto irq_free;
        }
    }

    memset(priv->tx_dma, 0, dma_desc_tx * sizeof(struct dma_desc));
    memset(priv->rx_dma, 0, dma_desc_rx * sizeof(struct dma_desc));

    sunxi_gmac_desc_init_chain(priv->tx_dma, (unsigned long)priv->dma_tx_phy, dma_desc_tx);
    sunxi_gmac_desc_init_chain(priv->rx_dma, (unsigned long)priv->dma_rx_phy, dma_desc_rx);

    priv->rx_clean = 0;
    priv->rx_dirty = 0;
    priv->tx_clean = 0;
    priv->tx_dirty = 0;

    sunxi_gmac_rx_refill(dev);

    /* extra statistics */
    memset(&priv->xstats, 0, sizeof(struct sunxi_gmac_extra_stats));

    phy_start(dev->phydev);

    sunxi_gmac_start_rx(priv, (unsigned long)((struct dma_desc *)priv->dma_rx_phy + priv->rx_dirty));
    sunxi_gmac_start_tx(priv, (unsigned long)((struct dma_desc *)priv->dma_tx_phy + priv->tx_dirty));

    napi_enable(&priv->napi);
    netif_start_queue(dev);
    
    /* Enable the Rx/Tx */
    sunxi_gmac_enable(priv);

    return 0;

irq_free:
    free_irq(dev->irq, dev);
out:
    phy_disconnect(dev->phydev);
disable_clk:
    sunxi_gmac_clk_disable(priv);
power_off:
    sunxi_gmac_power_off(priv);

    return ret;
}

static void sunxi_gmac_free_rx_skb(struct sunxi_gmac_priv *priv)
{
    int i;
    struct dma_desc *desc;

    for (i = 0; i < dma_desc_rx; i++) {
        if (priv->rx_skb[i] != NULL) {
            desc = priv->rx_dma + i;
            dma_unmap_single(priv->dev, (u32)desc_buf_get_addr(desc),
            desc_buf_get_len(desc),
            DMA_FROM_DEVICE);
            dev_kfree_skb_any(priv->rx_skb[i]);
            priv->rx_skb[i] = NULL;
        }
    }
}

static void sunxi_gmac_free_tx_skb(struct sunxi_gmac_priv *priv)
{
    int i;
    struct dma_desc *desc;
    int addr;

    for (i = 0; i < dma_desc_tx; i++) {
        if (priv->tx_skb[i] != NULL) {
            desc = priv->tx_dma + i;
            if ((addr = desc_buf_get_addr(desc))) {
                dma_unmap_single(priv->dev, (u32)addr,
                desc_buf_get_len(desc),
                DMA_TO_DEVICE);
            }
            dev_kfree_skb_any(priv->tx_skb[i]);
            priv->tx_skb[i] = NULL;
        }
    }
}

static void sunxi_gmac_free_dma_desc(struct sunxi_gmac_priv *priv)
{
    dma_free_coherent(priv->dev, dma_desc_rx * sizeof(struct dma_desc),
            priv->tx_dma, priv->dma_rx_phy);
    dma_free_coherent(priv->dev, dma_desc_tx * sizeof(struct dma_desc),
            priv->tx_dma, priv->dma_tx_phy);

    kfree(priv->rx_skb);
    kfree(priv->tx_skb);
}

static int sunxi_gmac_stop(struct net_device *dev)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);

    netif_stop_queue(dev);
    napi_disable(&priv->napi);

    netif_carrier_off(dev);

    sunxi_gmac_stop_rx(priv);
    sunxi_gmac_stop_tx(priv);

    phy_disconnect(dev->phydev);

    free_irq(dev->irq, dev);

    sunxi_gmac_disable(priv);

    netif_tx_lock_bh(dev);

    sunxi_gmac_free_rx_skb(priv);
    sunxi_gmac_free_tx_skb(priv);

    netif_tx_unlock_bh(dev);

    if (!priv->is_suspend) {
        sunxi_gmac_free_dma_desc(priv);
    }

    sunxi_gmac_clk_disable(priv);
    sunxi_gmac_power_off(priv);

    return 0;
}

static int sunxi_gmac_change_mtu(struct net_device *dev, int mtu)
{
    int max_mtu;

    if (netif_running(dev)) {
        return -EBUSY;
    }

    max_mtu = SKB_MAX_HEAD(NET_SKB_PAD + NET_IP_ALIGN);

    if ((mtu < 46) || (mtu > max_mtu))
        return -EINVAL;

    dev->mtu = mtu;

    netdev_update_features(dev);

    return 0;
}

static void sunxi_gmac_tx_poll(struct sunxi_gmac_priv *priv)
{
    u32 value;
    value = ioread32(priv->mem[0] + SUNXI_GMAC_TX_CTL1);
    iowrite32(value | 0x80000000, priv->mem[0] + SUNXI_GMAC_TX_CTL1);
}

static int sunxi_gmac_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    u32 entry;
    struct dma_desc *desc, *first;
    u32 len, temp_len = 0;
    int i, csum_insert;
    int nfrags = skb_shinfo(skb)->nr_frags;
    dma_addr_t paddr;

    if (unlikely(circ_space(priv->tx_dirty, priv->tx_clean, dma_desc_tx) < (nfrags + 1))) {
        if (!netif_queue_stopped(dev)) {
            netdev_err(dev, "%s: BUG Tx Ring full when queue awake\n", __func__);
            netif_stop_queue(dev);
        }
        return NETDEV_TX_BUSY;
    }

    csum_insert = (skb->ip_summed == CHECKSUM_PARTIAL);
    entry = priv->tx_dirty;
    first = priv->tx_dma + entry;
    desc = priv->tx_dma + entry;

    len = skb_headlen(skb);

    priv->tx_skb[entry] = skb;

    while(len != 0) {
        desc = priv->tx_dma + entry;
        temp_len = ((len > MAX_BUF_SZ) ? MAX_BUF_SZ : len);

        paddr = dma_map_single(priv->dev, skb->data, temp_len, DMA_TO_DEVICE);
        if (dma_mapping_error(priv->dev, paddr)) {
            dev_kfree_skb(skb);
            return -EIO;
        }

        sunxi_gmac_desc_buf_set(desc, paddr, temp_len);

        if (first != desc) {
            priv->tx_skb[entry] = NULL;
            sunxi_gmac_desc_set_own(desc);
        }

        entry = circ_inc(entry, dma_desc_tx);
        len -= temp_len;
    }

    for (i = 0; i < nfrags; i++) {
        const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

        len = skb_frag_size(frag);
        desc = priv->tx_dma + entry;
        paddr = skb_frag_dma_map(priv->dev, frag, 0, len, DMA_TO_DEVICE);
        if (dma_mapping_error(priv->dev, paddr)) {
            dev_kfree_skb(skb);
            return -EIO;
        }

        sunxi_gmac_desc_buf_set(desc, paddr, len);
        sunxi_gmac_desc_set_own(desc);
        priv->tx_skb[entry] = NULL;
        entry = circ_inc(entry, dma_desc_tx);
    }

    dev->stats.tx_bytes += skb->len;
    priv->tx_dirty = entry;

    sunxi_gmac_desc_tx_close(first, desc, csum_insert);

    sunxi_gmac_desc_set_own(first);

    if (circ_space(priv->tx_dirty, priv->tx_clean, dma_desc_tx) > SUNXI_GMAC_TX_THRESH) {
        netif_wake_queue(dev);
    }

    sunxi_gmac_tx_poll(priv);

    sunxi_gmac_tx_complete(priv);

    return NETDEV_TX_OK;
}

static netdev_features_t sunxi_gmac_fix_features(struct net_device *dev, netdev_features_t features)
{
    return features;
}

static void sunxi_gmac_hash_filter(struct sunxi_gmac_priv *priv, ulong lo, ulong hi)
{
    iowrite32(hi, priv->mem[0] + SUNXI_GMAC_RX_HASH0);
    iowrite32(lo, priv->mem[0] + SUNXI_GMAC_RX_HASH1);
}

static void sunxi_gmac_set_filter(struct sunxi_gmac_priv *priv, ulong flags)
{
    int tmp_flags = 0;

    tmp_flags |= ((flags >> 31) |
        ((flags >> 9) & 0x00000002) |
        ((flags << 1) & 0x00000010) |
        ((flags >> 3) & 0x00000060) |
        ((flags << 7) & 0x00000300) |
        ((flags << 6) & 0x00003000) |
        ((flags << 12) & 0x00030000) |
        (flags << 31));
    iowrite32(tmp_flags, priv->mem[0] + SUNXI_GMAC_RX_FRM_FLT);
}

static void sunxi_gmac_set_rx_mode(struct net_device *dev)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    u32 value = 0;

    netdev_dbg(dev, "mcasts %d, # unicast %d\n",
        netdev_mc_count(dev), netdev_uc_count(dev));

    spin_lock(&priv->lock);

    if (dev->flags & IFF_PROMISC) {
        value = SUNXI_GMAC_FRAME_FILTER_PR;
    } else if ((netdev_mc_count(dev) > SUNXI_GMAC_HASH_TABLE_SIZE) ||
             (dev->flags & IFF_ALLMULTI)) {
        value = SUNXI_GMAC_FRAME_FILTER_PM;
        sunxi_gmac_hash_filter(priv, ~0UL, ~0UL);
    } else if (!netdev_mc_empty(dev)) {
        u32 mc_filter[2];
        struct netdev_hw_addr *ha;

        value = SUNXI_GMAC_FRAME_FILTER_HMC;
        memset(mc_filter, 0, sizeof(mc_filter));
        netdev_for_each_mc_addr(ha, dev) {
            int bit_nr = bitrev32(~crc32_le(~0, ha->addr, 6)) >> 26;
            mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
        }

        sunxi_gmac_hash_filter(priv, mc_filter[0], mc_filter[1]);
    }

    if (netdev_uc_count(dev) > 16) {
        value |= SUNXI_GMAC_FRAME_FILTER_PR;
    } else {
        int reg = 1;
        struct netdev_hw_addr *ha;

        netdev_for_each_uc_addr(ha, dev) {
            sunxi_gmac_set_umac(priv, ha->addr, reg);
            reg++;
        }
    }

    sunxi_gmac_set_filter(priv, value);

    spin_unlock(&priv->lock);
}

static void sunxi_gmac_tx_timeout(struct net_device *dev)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);

    sunxi_gmac_tx_err(priv);
}

static int sunxi_gmac_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
    if (!netif_running(dev))
        return -EINVAL;

    if (!dev->phydev)
        return -ENODEV;

    return phy_mii_ioctl(dev->phydev, req, cmd);
}

static int sunxi_gmac_set_config(struct net_device *dev, struct ifmap *map)
{
    if (dev->flags & IFF_UP)
        return -EBUSY;

    if (map->base_addr != dev->base_addr) {
        return -EOPNOTSUPP;
    }

    if (map->irq != dev->irq) {
        return -EOPNOTSUPP;
    }

    return 0;
}

static int sunxi_gmac_set_mac_address(struct net_device *dev, void *p)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    struct sockaddr *addr = p;

    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;

    ether_addr_copy(dev->dev_addr, addr->sa_data);
    sunxi_gmac_set_umac(priv, dev->dev_addr, 0);
    return 0;
}

static void sunxi_gmac_loopback(struct sunxi_gmac_priv *priv, int en)
{
    int reg;
    reg = ioread32(priv->mem[0] + SUNXI_GMAC_BASIC_CTL0);
    if (en)
        reg |= 0x02;
    else
        reg &= ~0x02;

    iowrite32(reg, priv->mem[0] + SUNXI_GMAC_BASIC_CTL0);
}

static int sunxi_gmac_set_features(struct net_device *dev, netdev_features_t features)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);

    if (features & NETIF_F_LOOPBACK && netif_running(dev))
        sunxi_gmac_loopback(priv, 1);
    else
        sunxi_gmac_loopback(priv, 0);

    return 0;
}

static const struct net_device_ops sunxi_gmac_ops = {
    .ndo_open = sunxi_gmac_open,
    .ndo_stop = sunxi_gmac_stop,
    .ndo_change_mtu = sunxi_gmac_change_mtu,
    .ndo_start_xmit = sunxi_gmac_xmit,
    .ndo_fix_features = sunxi_gmac_fix_features,
    .ndo_set_rx_mode = sunxi_gmac_set_rx_mode,
    .ndo_tx_timeout = sunxi_gmac_tx_timeout,
    .ndo_do_ioctl = sunxi_gmac_ioctl,
    .ndo_set_config = sunxi_gmac_set_config,
    .ndo_set_mac_address = sunxi_gmac_set_mac_address,
    .ndo_set_features = sunxi_gmac_set_features,
};

static int sunxi_gmac_check_if_running(struct net_device *dev)
{
    if (!netif_running(dev))
        return -EBUSY;

    return 0;
}

static int sunxi_gmac_ethtool_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    struct phy_device *phydev = dev->phydev;
    int rc;

    if (phydev == NULL) {
        netdev_err(dev, "%s: PHY is not register\n", dev->name);
        return -ENODEV;
    }

    if (!netif_running(dev)) {
        netdev_err(dev, "interface is disabled: we cannot track\n");
        return -EBUSY;
    }

    cmd->transceiver = XCVR_INTERNAL;
    spin_lock_irq(&priv->lock);
    rc = phy_ethtool_gset(phydev, cmd);
    spin_unlock_irq(&priv->lock);
    return rc;
}

static int sunxi_gmac_ethtool_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);
    struct phy_device *phydev = dev->phydev;
    int rc;

    spin_lock(&priv->lock);
    rc = phy_ethtool_sset(phydev, cmd);
    spin_unlock(&priv->lock);

    return rc;
}

static int sunxi_gmac_ethtool_get_sset_count(struct net_device *dev, int sset)
{
    int len;

    switch(sset) {
        case ETH_SS_STATS:
            len = 0;
            return len;
        default:
            return -EOPNOTSUPP;
    }
}

static void sunxi_gmac_ethtool_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
    strlcpy(info->driver, "sunxi_gmac", sizeof(info->driver));
    strcpy(info->version, "SUNXI Gbgit driver ng");
    info->fw_version[0] = '\0';
}

static void sunxi_gmac_ethtool_get_wol(struct net_device *dev, struct ethtool_wolinfo *info)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);

    if (priv->phy_ext && dev->phydev->drv->get_wol) {
        phy_ethtool_get_wol(dev->phydev, info);
    }
}

static int sunxi_gmac_ethtool_set_wol(struct net_device *dev, struct ethtool_wolinfo *info)
{
    struct sunxi_gmac_priv *priv = netdev_priv(dev);

    if (!(dev->flags & IFF_UP)) {
        netdev_err(dev, "device not up\n");
        return -EINVAL;
    }

    if (priv->phy_ext) {
        return phy_ethtool_set_wol(dev->phydev, info);
    }

    return -EOPNOTSUPP;
}

static const struct ethtool_ops sunxi_gmac_ethool_ops = {
    .begin = sunxi_gmac_check_if_running,
    .get_settings = sunxi_gmac_ethtool_get_settings,
    .set_settings = sunxi_gmac_ethtool_set_settings,
    .get_link = ethtool_op_get_link,
    .get_sset_count = sunxi_gmac_ethtool_get_sset_count,
    .get_drvinfo = sunxi_gmac_ethtool_get_drvinfo,
    .get_wol = sunxi_gmac_ethtool_get_wol,
    .set_wol = sunxi_gmac_ethtool_set_wol,
};

static int sunxi_gmac_tx_ls(struct dma_desc *desc)
{
    return desc->desc0.all & 0x80000000;
}

static int sunxi_gmac_desc_tx_status(struct dma_desc *desc, struct sunxi_gmac_extra_stats *stats)
{
    int ret = 0;

    if (desc->desc0.tx.under_err) {
        stats->tx_underflow++;
        ret = -1;
    }

    if (desc->desc0.tx.no_carr) {
        stats->tx_carrier++;
        ret = -1;
    }

    if (desc->desc0.tx.loss_carr) {
        stats->tx_losscarrier++;
        ret = -1;
    }

    if (desc->desc0.tx.deferred) {
        stats->tx_deferred++;
    }

    return ret;
}

static int sunxi_gmac_desc_rx_status(struct dma_desc *desc, struct sunxi_gmac_extra_stats *stats)
{
    int ret = good_frame;

    if (desc->desc0.rx.last_desc == 0) {
        return discard_frame;
    }

    if (desc->desc0.rx.err_sum) {
        if (desc->desc0.rx.desc_err)
            stats->rx_desc++;

        if (desc->desc0.rx.sou_filter)
            stats->sa_filter_fail++;

        if (desc->desc0.rx.over_err)
            stats->overflow_error++;

        if (desc->desc0.rx.ipch_err)
            stats->ipc_csum_error++;

        if (desc->desc0.rx.late_coll)
            stats->rx_collision++;

        if (desc->desc0.rx.crc_err)
            stats->rx_crc++;

        ret = discard_frame;
    }

    if (desc->desc0.rx.len_err) {
        ret = discard_frame;
    }

    if (desc->desc0.rx.mii_err) {
        ret = discard_frame;
    }

    return ret;
}

static void sunxi_gmac_tx_complete(struct sunxi_gmac_priv *priv)
{
    u32 entry = 0;
    struct sk_buff *skb = NULL;
    struct dma_desc *desc = NULL;
    int tx_stat;

    while(circ_cnt(priv->tx_dirty, priv->tx_clean, dma_desc_tx) > 0) {
        entry = priv->tx_clean;
        desc = priv->tx_dma + entry;

        if (sunxi_gmac_desc_get_own(desc))
            break;

        if (sunxi_gmac_tx_ls(desc)) {
            tx_stat = sunxi_gmac_desc_tx_status(desc, &priv->xstats);
            if (likely(!tx_stat))
                priv->netdev->stats.tx_packets++;
            else
                priv->netdev->stats.tx_errors++;
        }

        dma_unmap_single(priv->dev, (u32)desc_buf_get_addr(desc),
                                    desc_buf_get_len(desc), DMA_TO_DEVICE);

        skb = priv->tx_skb[entry];
        priv->tx_skb[entry] = NULL;
        sunxi_gmac_desc_init(desc);

        priv->tx_clean = circ_inc(entry, dma_desc_tx);

        if (unlikely(skb == NULL))
            continue;

        dev_kfree_skb_any(skb);
    }

    if (unlikely(netif_queue_stopped(priv->netdev) &&
        (circ_space(priv->tx_dirty, priv->tx_clean, dma_desc_tx)) > SUNXI_GMAC_TX_THRESH)) {
            netif_wake_queue(priv->netdev);
        }
}

int sunxi_gmac_desc_rx_frame_len(struct dma_desc *desc)
{
    return desc->desc0.rx.frm_len;
}

static int sunxi_gmac_rx(struct sunxi_gmac_priv *priv, int limit)
{
    u32 rxcount = 0;
    u32 entry;
    struct dma_desc *desc;
    struct sk_buff *skb;
    int status;
    int frame_len;

    while(rxcount < limit) {
        entry = priv->rx_dirty;
        desc = priv->rx_dma + entry;

        if (sunxi_gmac_desc_get_own(desc))
            break;

        rxcount++;
        priv->rx_dirty = circ_inc(priv->rx_dirty, dma_desc_rx);

        frame_len = sunxi_gmac_desc_rx_frame_len(desc);
        status = sunxi_gmac_desc_rx_status(desc, &priv->xstats);

        netdev_dbg(priv->netdev, "Rx frame size %d, status:%d\n", frame_len, status);

        skb = priv->rx_skb[entry];

        if (unlikely(!skb)) {
            netdev_err(priv->netdev, "Skb is null\n");
            priv->netdev->stats.rx_dropped++;
            break;
        }

        if (status == discard_frame) {
            netdev_dbg(priv->netdev, "Get error pkt\n");
            priv->netdev->stats.rx_errors++;
            continue;
        }

        if (unlikely(status != llc_snap))
            frame_len -= ETH_FCS_LEN;

        priv->rx_skb[entry] = NULL;

        skb_put(skb, frame_len);

        dma_unmap_single(priv->dev, (u32)desc_buf_get_addr(desc),
                        desc_buf_get_len(desc), DMA_FROM_DEVICE);

        skb->protocol = eth_type_trans(skb, priv->netdev);

        skb->ip_summed = CHECKSUM_UNNECESSARY;
        napi_gro_receive(&priv->napi, skb);

        priv->netdev->stats.rx_packets++;
        priv->netdev->stats.rx_bytes += frame_len;
    }

    sunxi_gmac_rx_refill(priv->netdev);

    return rxcount;
}

static int sunxi_gmac_poll(struct napi_struct *napi, int budget)
{
    struct sunxi_gmac_priv *priv = container_of(napi, struct sunxi_gmac_priv, napi);
    int work_done = 0;

    sunxi_gmac_tx_complete(priv);
    work_done = sunxi_gmac_rx(priv, budget);

    if (work_done < budget) {
        napi_complete(napi);
        sunxi_gmac_int_enable(priv);
    }

    return work_done;
}

static int sunxi_gmac_parse_dt(struct platform_device *pdev)
{
    struct net_device *netdev = platform_get_drvdata(pdev);
    struct sunxi_gmac_priv *priv = netdev_priv(netdev);
    struct device_node *np = pdev->dev.of_node;

    char buf[64] = {0};
    int ret = -EINVAL;
    const char *power;
    int i;

    for (i = 0; i < 3; i++) {
        priv->res[i] = platform_get_resource(pdev, IORESOURCE_MEM, i);
        if (!priv->res[i]) {
            goto out;
        }
    }

    netdev->irq = platform_get_irq_byname(pdev, "gmacirq");
    if (netdev->irq == -ENXIO) {
        dev_err(&pdev->dev, "MAC IRQ not found\n");
        ret = netdev->irq;
        goto out;
    }

    priv->clk[CLK_GMAC] = of_clk_get_by_name(np, "gmac");
    if (IS_ERR(priv->clk[CLK_GMAC]) || !priv->clk[CLK_GMAC]) {
        dev_err(&pdev->dev, "gmac clk not found\n");
        goto out;
    }

    if (of_parse_phandle(np, "phy-handle", 0)) {
        priv->phy_ext = EXT_PHY;
        for (i = 0; i < POWER_CHAN_NUM; i++) {
            snprintf(buf, sizeof(buf)-1, "gmac-power%d", i);
            ret = of_property_read_string(pdev->dev.of_node, buf, &power);
            if(ret) {
                priv->power[i] = NULL;
                dev_info(&pdev->dev, "gmac-power%d:NULL\n", i);
                continue;
            }
            priv->power[i] = regulator_get(NULL, power);
            if (IS_ERR(priv->power[i])) {
                dev_err(&pdev->dev, "gmac-power%d get error\n", i);
                ret = -EINVAL;
                goto out;
            }
        }
    } else {
        priv->phy_ext = INT_PHY;
        if (of_property_read_bool(np, "use_ephy25m")) {
            priv->clk[CLK_EPHY] = of_clk_get_by_name(pdev->dev.of_node, "ephy");
            if (!priv->clk[CLK_EPHY] || IS_ERR(priv->clk[CLK_EPHY])) {
                dev_err(&pdev->dev, "ephy clk not found\n");
                goto out;
            }
        }
    }

    priv->phy_interface = of_get_phy_mode(np);
    switch(priv->phy_interface) {
        case PHY_INTERFACE_MODE_MII:
        case PHY_INTERFACE_MODE_RGMII:
        case PHY_INTERFACE_MODE_RMII:
            break;
        default:
            dev_err(&pdev->dev, "Not support phy type\n");
            priv->phy_interface = PHY_INTERFACE_MODE_MII;
            break;
    }

    if (of_property_read_u32(np, "tx-delay", &priv->tx_delay)) {
        ret = -EINVAL;
        goto out;
    }

    if (of_property_read_u32(np, "rx-delay", &priv->rx_delay)) {
        ret = -EINVAL;
        goto out;
    }

    return 0;

out:
    return ret;
}

static int sunxi_gmac_select_gpio_state(struct sunxi_gmac_priv *priv, const char *name)
{
    int ret = 0;
    struct pinctrl_state *state = NULL;

    state = pinctrl_lookup_state(priv->pinctrl, name);
    if (IS_ERR(state)) {
        dev_err(priv->dev, "lookup state(%s) failed\n", name);
        return -EINVAL;
    }

    ret = pinctrl_select_state(priv->pinctrl, state);

    if(ret < 0) {
        dev_err(priv->dev, "pinctrl select state(%s) failed\n", name);
        return ret;
    }

    return ret;
}

static int sunxi_gmac_hw_init(struct net_device *netdev)
{
    int i;
    int ret = -EINVAL;
    struct sunxi_gmac_priv *priv = netdev_priv(netdev);

    for (i = 0; i < 3; i++) {
        priv->mem[i] = devm_ioremap_resource(priv->dev, priv->res[i]);
        if (IS_ERR(priv->mem[i])) {
            dev_err(priv->dev, "ioremap %d failed\n", i);
	    ret = PTR_ERR(priv->mem[i]);
            goto err;
        }
    }

    if (priv->phy_ext) {
        priv->pinctrl = devm_pinctrl_get_select_default(priv->dev);
        if (IS_ERR_OR_NULL(priv->pinctrl)) {
	    ret = PTR_ERR(priv->pinctrl);
            dev_err(priv->dev, "pinctrl error!\n");
            priv->pinctrl = NULL;
            goto err;
        }
    }

    return 0;
err:
    while(i-- > 0) {
        devm_iounmap(priv->dev, priv->mem[i]);
    }
    return ret;
}

static int sunxi_gmac_power_on(struct sunxi_gmac_priv *priv)
{
    int v, i;

    v = ioread32(priv->mem[2]);
    if (priv->phy_ext) {
        v &= ~(1 << 15);

        for (i = 0; i < POWER_CHAN_NUM; i++) {
            if (IS_ERR_OR_NULL(priv->power[i])) {
                continue;
            }

            if (regulator_enable(priv->power[i])) {
                dev_err(priv->dev, "gmac-power%d enable error\n", i);
                return -EINVAL;
            }
        }
    } else {
        v |= (1 << 15);
        v &= ~(1 << 16);
        v |= (3 << 17);
    }

    iowrite32(v, priv->mem[2]);

    return 0;
}

static void sunxi_gmac_power_off(struct sunxi_gmac_priv *priv)
{
    int v, i;

    if (priv->phy_ext) {
        for (i = 0; i < POWER_CHAN_NUM; i++) {
            if (IS_ERR_OR_NULL(priv->power[i]))
                continue;
            regulator_disable(priv->power[i]);
        }
    } else {
        v = ioread32(priv->mem[2]);
        v |= (1 << 16);
        iowrite32(v, priv->mem[2]);
    }
}

static void sunxi_gmac_clk_enable(struct sunxi_gmac_priv *priv)
{
    u32 clk_val;
    phy_interface_t phy_interface;
    // u32 efuse_val;

    if (clk_prepare_enable(priv->clk[CLK_GMAC])){
        netdev_err(priv->netdev, "try to enable gmac clk failed\n");
        return;
    }

    if (!IS_ERR_OR_NULL(priv->clk[CLK_EPHY])) {
        if (clk_prepare_enable(priv->clk[CLK_EPHY])) {
            netdev_err(priv->netdev, "try to enable ephy clk failed\n");
            clk_disable_unprepare(priv->clk[CLK_GMAC]);
            return;
        }
    }

    phy_interface = priv->phy_interface;

    clk_val = ioread32(priv->mem[2]);

    if (phy_interface == PHY_INTERFACE_MODE_RGMII)
        clk_val |= 0x00000004;
    else
        clk_val &= (~0x00000004);

    clk_val &= (~0x00002003);
    if (phy_interface == PHY_INTERFACE_MODE_RGMII
			|| phy_interface == PHY_INTERFACE_MODE_GMII)
		clk_val |= 0x00000002;
	else if (phy_interface == PHY_INTERFACE_MODE_RMII)
		clk_val |= 0x00002001;

    if (priv->phy_ext == INT_PHY) {

    }
    /* Adjust Tx/Rx clock delay */
    clk_val &= ~(0x07 << 10);
	clk_val |= ((priv->tx_delay & 0x07) << 10);
	clk_val &= ~(0x1F << 5);
	clk_val |= ((priv->rx_delay & 0x1F) << 5);

    iowrite32(clk_val, priv->mem[2]);
}

static void sunxi_gmac_clk_disable(struct sunxi_gmac_priv *priv)
{
    int i;

    for (i = 0; i < 2; i++) {
        if (!IS_ERR_OR_NULL(priv->clk[i])) {
            clk_disable_unprepare(priv->clk[i]);
        }
    }
}

static void sunxi_chip_hwaddr(uint8_t *addr)
{
#define MD5_SIZE	16
#define CHIP_SIZE	16

	struct crypto_ahash *tfm;
	struct ahash_request *req;
	struct scatterlist sg;
	u8 result[MD5_SIZE];
	u8 chipid[CHIP_SIZE];
	int i = 0;
	int ret = -1;

	memset(chipid, 0, sizeof(chipid));
	memset(result, 0, sizeof(result));
	tfm = crypto_alloc_ahash("md5", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		pr_err("Failed to alloc md5\n");
		return;
	}

	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!req)
		goto out;

	ahash_request_set_callback(req, 0, NULL, NULL);

	ret = crypto_ahash_init(req);
	if (ret) {
		pr_err("crypto_ahash_init() failed\n");
		goto out;
	}

	sg_init_one(&sg, chipid, sizeof(chipid));
	ahash_request_set_crypt(req, &sg, result, sizeof(chipid));
	ret = crypto_ahash_update(req);
	if (ret) {
		pr_err("crypto_ahash_update() failed for id\n");
		goto out;
	}

	ret = crypto_ahash_final(req);
	if (ret) {
		pr_err("crypto_ahash_final() failed for result\n");
		goto out;
	}

	ahash_request_free(req);

	/* Choose md5 result's [0][2][4][6][8][10] byte as mac address */
	for (i = 0; i < ETH_ALEN; i++)
		addr[i] = result[2 * i];
	addr[0] &= 0xfe; /* clear multicast bit */
	addr[0] |= 0x02; /* set local assignment bit (IEEE802) */

out:
    crypto_free_ahash(tfm);
}

static void sunxi_gmac_check_addr(struct net_device *netdev, char *addr)
{
    int i;
    char *p = addr;

    if (!is_valid_ether_addr(netdev->dev_addr)) {
        for (i = 0; i < ETH_ALEN; i++, p++)
            netdev->dev_addr[i] = simple_strtoul(p, &p, 16);

        if (!is_valid_ether_addr(netdev->dev_addr))
            sunxi_chip_hwaddr(netdev->dev_addr);

        if (!is_valid_ether_addr(netdev->dev_addr)) {
            random_ether_addr(netdev->dev_addr);
            netdev_warn(netdev, "Use random mac address\n");
        }
    }
}

static const struct debugfs_reg32 sunxi_gmac_regs[] = {
    {
        .name = "basic_ctl0",
        .offset = 0x00,
    },{
        .name = "basic_ctl1",
        .offset = 0x04,
    }
};

static const struct debugfs_reg32 sunxi_gmac_rgmii[] = {
    {
        .name = "sgmii",
        .offset = 0x80,
    },
};

static struct debugfs_regset32 sunxi_gmac_regset = {
    .regs = sunxi_gmac_regs,
    .nregs = ARRAY_SIZE(sunxi_gmac_regs),
};

static struct debugfs_regset32 sunxi_gmac_rgmii_regset = {
    .regs = sunxi_gmac_rgmii,
    .nregs = ARRAY_SIZE(sunxi_gmac_rgmii),
};

static int sunxi_gmac_probe(struct platform_device *pdev)
{
    struct sunxi_gmac_priv *priv;
    struct net_device *netdev;
    struct phy_device *phydev;
    int ret;

#ifdef CONFIG_OF
	pdev->dev.dma_mask = &sunxi_gmac_dma_mask;
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
#endif

    netdev = alloc_etherdev(sizeof(*priv));
    if (!netdev)
        return -ENOMEM;

    SET_NETDEV_DEV(netdev, &pdev->dev);

    priv = netdev_priv(netdev);
    priv->netdev = netdev;
    priv->dev = &pdev->dev;
    priv->np = pdev->dev.of_node;
    platform_set_drvdata(pdev, netdev);

    ether_setup(netdev);
    netdev->netdev_ops = &sunxi_gmac_ops;
    netdev_set_default_ethtool_ops(netdev, &sunxi_gmac_ethool_ops);

    netdev->hw_features =  NETIF_F_SG | NETIF_F_HIGHDMA | NETIF_F_IP_CSUM |
				NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM;
    netdev->hw_features |= NETIF_F_LOOPBACK;
    netdev->priv_flags |= IFF_UNICAST_FLT;

    netdev->watchdog_timeo = msecs_to_jiffies(watchdog);
    netdev->dev.parent = &pdev->dev;

    ret = sunxi_gmac_parse_dt(pdev);
    if (ret) {
        goto reg_err;
    }

    ret = sunxi_gmac_hw_init(netdev);
    if(ret) {
        goto reg_err;
    }

    netif_napi_add(netdev, &priv->napi, sunxi_gmac_poll, BUDGET);

    spin_lock_init(&priv->lock);

    ret = register_netdev(netdev);
    if (ret) {
        netif_napi_del(&priv->napi);
        dev_err(&pdev->dev, "Register %s failed\n", netdev->name);
        goto reg_err;
    }

    netif_carrier_off(netdev);

    device_enable_async_suspend(&pdev->dev);

    priv->debugfs_dir = debugfs_create_dir(dev_name(&pdev->dev), NULL);
    
    sunxi_gmac_regset.base = priv->mem[0];
    sunxi_gmac_rgmii_regset.base = priv->mem[1];

    priv->debugfs_regs = debugfs_create_regset32("regs", 0644, priv->debugfs_dir, &sunxi_gmac_regset);
    priv->debugfs_rgmii = debugfs_create_regset32("rgmii", 0644,  priv->debugfs_dir, &sunxi_gmac_rgmii_regset);

    sunxi_gmac_check_addr(netdev, mac_str);
    

    return 0;
unregister:
    unregister_netdev(netdev);
reg_err:
    free_netdev(netdev);
    return ret;
}

static int sunxi_gmac_remove(struct platform_device *pdev)
{
    struct net_device *netdev = platform_get_drvdata(pdev);
    struct sunxi_gmac_priv *priv = netdev_priv(netdev);
    int i;

    if (!IS_ERR_OR_NULL(netdev->phydev))
        phy_disconnect(netdev->phydev);

    debugfs_remove(priv->debugfs_regs);
    debugfs_remove(priv->debugfs_rgmii);
    debugfs_remove(priv->debugfs_dir);

     if (priv->phy_ext) {
        if (!IS_ERR_OR_NULL(priv->pinctrl))
            devm_pinctrl_put(priv->pinctrl);
    }

    for (i = 0; i < 2; i++) {
        if (!IS_ERR_OR_NULL(priv->clk[i]))
            clk_put(priv->clk[i]);
    }

    for(i = 0; i < 3; i++) {
        devm_iounmap(&pdev->dev, priv->mem[i]);
    }

    unregister_netdev(netdev);

    free_netdev(netdev);

    return 0;
}

static struct platform_driver sunxi_gmac_driver = {
    .driver = {
        .name = "sunxi_gmac",
        .of_match_table = sunxi_gmac_ids,
    },
    .probe = sunxi_gmac_probe,
    .remove = sunxi_gmac_remove,
};

module_platform_driver(sunxi_gmac_driver);
MODULE_AUTHOR("jeck.chen@dbappsecurity.com.cn");
MODULE_LICENSE("GPL");
