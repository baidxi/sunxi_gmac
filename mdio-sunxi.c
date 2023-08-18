#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/mdio.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>

#define MII_BUSY 1
#define MDIO_DATA 0x4
#define MII_WRITE 2
#define MII_READ 0

struct sunxi_mdio_priv {
    struct mii_bus *bus;
    void __iomem *base;
    struct device *dev;
    struct resource *res;
    uint32_t mdc_div;
    struct pinctrl *pinctrl;
};

static const struct of_device_id sunxi_mdio_ids[] = {
    { .compatible = "allwinner,sunxi-mdio"},
    {},
};

MODULE_DEVICE_TABLE(of, sunxi_mdio_ids);

static int sunxi_mdio_reg_read(struct sunxi_mdio_priv *sunxi_mdio, int addr, int reg)
{
    uint32_t value = 0;
    int timeout = 10;

    value |= ((sunxi_mdio->mdc_div & 0x07) << 20);
    value |= (((addr << 12) & (0x0001f000)) |
             ((reg << 4) & ( 0x7f0)) | MII_BUSY);

    while(((ioread32(sunxi_mdio->base)) & MII_BUSY) == 1)
        ;

    iowrite32(value, sunxi_mdio->base);
    do {
	if (((ioread32(sunxi_mdio->base)) & MII_BUSY) == 0) {
	     break;
	}
	msleep(1);
	timeout--;
    }while(timeout);

    if (!timeout)
	return -EBUSY;

    return ioread32(sunxi_mdio->base + MDIO_DATA);
}


static int sunxi_mdio_reg_write(struct sunxi_mdio_priv *sunxi_mdio, int addr, int reg, u16 val)
{
    uint32_t value;
    int timeout = 10;

    value = ((sunxi_mdio->mdc_div << 20) |
             ((addr << 12) & 0x0001F000) |
             ((reg << 4) & 0x000007F0) | MII_WRITE | MII_BUSY);

    do {
	if (((ioread32(sunxi_mdio->base)) & MII_BUSY) == 0) {
	   break;
	} 
	msleep(1);
	timeout--;
    }while(timeout);

     if (!timeout) 
	return -EBUSY;

    iowrite32(val, sunxi_mdio->base + MDIO_DATA);
    iowrite32(value, sunxi_mdio->base);

    do {
	if (((ioread32(sunxi_mdio->base)) & MII_BUSY) == 0) {
	   break;
	} 
	msleep(1);
	timeout--;
    }while(timeout);

    if (!timeout) 
	return -EBUSY;

    return 0;
}
static int sunxi_mdio_read(struct mii_bus *bus, int addr, int reg)
{
    struct sunxi_mdio_priv *sunxi_mdio = bus->priv;

    return sunxi_mdio_reg_read(sunxi_mdio, addr, reg);
}



static int sunxi_mdio_write(struct mii_bus *bus, int addr, int reg, u16 val)
{
    struct sunxi_mdio_priv *sunxi_mdio = bus->priv;
    return sunxi_mdio_reg_write(sunxi_mdio, addr, reg, val);

}

static int sunxi_mdio_reset(struct mii_bus *bus)
{
    struct sunxi_mdio_priv *sunxi_mdio = bus->priv;

    iowrite32((4 << 2), sunxi_mdio->base);

    return 0;
}

static int sunxi_mdio_dt_parse(struct sunxi_mdio_priv *priv, struct platform_device *pdev)
{
    if (of_property_read_u32(pdev->dev.of_node, "mdc-div", &priv->mdc_div)) {
        dev_err(&pdev->dev, "mdc_div not found\n");
        return -EINVAL;
    }

    priv->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    if(!priv->res){
        return -EINVAL;
    }

    priv->base = devm_ioremap_resource(&pdev->dev, priv->res);

    if(IS_ERR(priv->base)) {
        return PTR_ERR(priv->base);
    }

    return 0;
}

static int sunxi_mdio_select_gpio_state(struct sunxi_mdio_priv *sunxi_mdio, const char *name)
{
    int ret = 0;
    struct pinctrl_state *state = NULL;

    state = pinctrl_lookup_state(sunxi_mdio->pinctrl, name);

    if (IS_ERR(state)) {
        dev_err(sunxi_mdio->dev, "lookup pinctrl state failed\n");
        return -EINVAL;
    }

    ret = pinctrl_select_state(sunxi_mdio->pinctrl, state);
    if (ret < 0) {
        dev_err(sunxi_mdio->dev, "pinctrl select state (%s) failed:%d\n", name, ret);
        return ret;
    }

    return 0;
}

static int sunxi_mdio_probe(struct platform_device *pdev)
{
    struct sunxi_mdio_priv *sunxi_mdio;
    struct mii_bus *mii_bus;
    int ret;

    mii_bus = mdiobus_alloc_size(sizeof(*sunxi_mdio));

    if (!mii_bus)
        return -ENOMEM;

    mii_bus->name = dev_name(&pdev->dev);
    mii_bus->owner = THIS_MODULE;

    mii_bus->read = sunxi_mdio_read;
    mii_bus->write = sunxi_mdio_write;
    mii_bus->reset = sunxi_mdio_reset;
    snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s-%x", mii_bus->name, 0);

    mii_bus->parent = &pdev->dev;

    sunxi_mdio = mii_bus->priv;
    sunxi_mdio->bus = mii_bus;
    sunxi_mdio->dev = &mii_bus->dev;

    if ((ret = sunxi_mdio_dt_parse(sunxi_mdio, pdev))) {
        dev_err(&pdev->dev, "parse dt failed:%d\n", ret);
        goto err;
    }

    iowrite32((3 << 20), sunxi_mdio->base);
    
    sunxi_mdio->pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

    if (IS_ERR_OR_NULL(sunxi_mdio->pinctrl)) {
        dev_err(&pdev->dev, "pinctrl not found\n");
        ret = -EINVAL;
        goto err;
    }

    if ((ret = of_mdiobus_register(mii_bus, pdev->dev.of_node))) {
        dev_err(&pdev->dev, "register %s mdio faled:%d\n", dev_name(&pdev->dev), ret);
        goto reg_err;
    }

    platform_set_drvdata(pdev, mii_bus);

    return 0;
reg_err:
    devm_pinctrl_put(sunxi_mdio->pinctrl);
err:
    kfree(mii_bus);
    return ret;
}

static int sunxi_mdio_remove(struct platform_device *pdev)
{
    struct mii_bus *bus = platform_get_drvdata(pdev);
    struct sunxi_mdio_priv *priv = bus->priv;
    struct phy_device *phydev;
    int i;

    mdiobus_unregister(bus);

    sunxi_mdio_select_gpio_state(priv, "sleep");

    devm_pinctrl_put(priv->pinctrl);

    kfree(bus);

    return 0;
}

struct platform_driver sunxi_mdio_driver = {
    .driver = {
        .name = "sunxi_mdio",
        .of_match_table = sunxi_mdio_ids,
    },
    .probe = sunxi_mdio_probe,
    .remove = sunxi_mdio_remove,
};

module_platform_driver(sunxi_mdio_driver);
MODULE_LICENSE("GPL");
