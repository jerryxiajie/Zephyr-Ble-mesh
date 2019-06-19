/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/printk.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <board.h>
#include <gpio.h>
#include <display/mb_display.h>

// #include "board.h"

/* Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)

static struct mb_image open = MB_IMAGE({ 0, 0, 1, 0, 0 },
					 { 0, 1, 1, 0, 0 },
					 { 0, 0, 1, 0, 0 },
					 { 0, 0, 1, 0, 0 },
					 { 0, 1, 1, 1, 0 });

static struct mb_image close = MB_IMAGE({ 0, 0, 0, 0, 0 },
					 { 0, 0, 0, 0, 0 },
					 { 0, 0, 0, 0, 0 },
					 { 0, 0, 0, 0, 0 },
					 { 0, 0, 0, 0, 0 });

static void gen_onoff_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static void gen_onoff_set_unack(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static struct bt_mesh_cfg_srv cfg_srv = {
	.relay = BT_MESH_RELAY_ENABLED,
	.beacon = BT_MESH_BEACON_ENABLED,
#if defined(CONFIG_BT_MESH_FRIEND)
	.frnd = BT_MESH_FRIEND_ENABLED,
#else
	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BT_MESH_GATT_PROXY)
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
	.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
	.default_ttl = 2,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),
	.relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};


BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv, NULL, 2 + 2);
// BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli, NULL, 2 + 2);


// static u8_t trans_id;
static u16_t primary_addr;
static u16_t primary_net_idx;


static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_GET, 0, gen_onoff_get },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET, 2, gen_onoff_set },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, 2, gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

struct onoff_state {
	u8_t current;
	u8_t previous;
};

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
//	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv, NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
	struct onoff_state *onoff_state = model->user_data;

	printk("addr 0x%04x onoff 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, onoff_state->current);
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
	net_buf_simple_add_u8(&msg, onoff_state->current);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send On Off Status response\n");
	}
}

static void gen_onoff_set_unack(struct bt_mesh_model *model,
				struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct net_buf_simple *msg = model->pub->msg;
	struct mb_display *disp = mb_display_get();
	int err;
	u8_t  temp;

	temp = net_buf_simple_pull_u8(buf);
	printk("Set_Unack: addr 0x%02x state 0x%02x temp %d\n",
    	   bt_mesh_model_elem(model)->addr, temp, temp);

	if (temp == 1)
	{
//		mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, K_FOREVER, &arrow, 1);
		mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, K_FOREVER,
			 &open, 1);
//		printk("Display\n");
//		mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
//			 K_FOREVER, "1");
	}
	else
	{
		mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, K_FOREVER,
			 &close, 1);
	}

}

static void gen_onoff_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	printk("gen_onoff_set\n");

	gen_onoff_set_unack(model, ctx, buf);
//	gen_onoff_get(model, ctx, buf);
}


// static int output_number(bt_mesh_output_action_t action, uint32_t number)
// {
// 	printk("OOB Number: %u\n", number);

// 	return 0;
// }

static void prov_complete(u16_t net_idx, u16_t addr)
{
	printk("provisioning complete for net_idx 0x%04x addr 0x%04x\n",
	       net_idx, addr);
	primary_addr = addr;
	primary_net_idx = net_idx;
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
#if 0
	.output_size = 4,
	.output_actions = (BT_MESH_DISPLAY_NUMBER),
	.output_number = output_number,
#else
	.output_size = 0,
	.output_actions = 0,
	.output_number = 0,
#endif
	.complete = prov_complete,
	.reset = prov_reset,
};

static void bt_ready(int err)
{
	struct bt_le_oob oob;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	/* Use identity address as device UUID */
	if (bt_le_oob_get_local(&oob)) {
		printk("Identity Address unavailable\n");
	} else {
		memcpy(dev_uuid, oob.addr.a.val, 6);
	}

	printk("UUID: dev_uuid = %d , %d , %d \n", dev_uuid[0],dev_uuid[1],dev_uuid[2]);

	/* This will be a no-op if settings_load() loaded provisioning info */
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	printk("Mesh initialized\n");
}

void main(void)
{
	int err;

	printk("Initializing...\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
}
