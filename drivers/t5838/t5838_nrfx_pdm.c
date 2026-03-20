#define DT_DRV_COMPAT nordic_nrf_pdm

#include "t5838.h"

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/clock/nrf-auxpll.h>
#include <dmm.h>
#include <nrfx_pdm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(t5838_nrfx_pdm, CONFIG_AUDIO_DMIC_LOG_LEVEL);

#ifdef CONFIG_AUDIO_DMIC_NRFX_PDM
BUILD_ASSERT(CONFIG_AUDIO_DMIC_NRFX_PDM == 0,
	     "Both T5838 and nrfx_pdm drivers enabled, set "
	     "CONFIG_AUDIO_DMIC_NRFX_PDM to false to use T5838 driver");
#endif

/** From here bellow this is mostly modified copy of zephyr nrf pdm driver */

#define NODE_AUDIO_AUXPLL DT_NODELABEL(audio_auxpll)
#define NODE_AUDIOPLL     DT_NODELABEL(audiopll)

#if CONFIG_SOC_SERIES_NRF54HX
#define DMIC_NRFX_CLOCK_FREQ MHZ(16)
#define DMIC_NRFX_AUDIO_CLOCK_FREQ DT_PROP_OR(NODE_AUDIOPLL, frequency, 0)
#elif DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
#define AUXPLL_FREQUENCY_SETTING DT_PROP(NODE_AUDIO_AUXPLL, nordic_frequency)
BUILD_ASSERT((AUXPLL_FREQUENCY_SETTING == NRF_AUXPLL_FREQ_DIV_AUDIO_48K) ||
             (AUXPLL_FREQUENCY_SETTING == NRF_AUXPLL_FREQ_DIV_AUDIO_44K1),
              "Unsupported Audio AUXPLL frequency selection for PDM");

#define DMIC_T5838_AUDIO_CLOCK_FREQ CLOCK_CONTROL_NRF_AUXPLL_GET_FREQ(NODE_AUDIO_AUXPLL)

#define DMIC_T5838_CLOCK_FREQ MHZ(32)

#else
#define DMIC_T5838_CLOCK_FREQ MHZ(32)
#define DMIC_T5838_AUDIO_CLOCK_FREQ DT_PROP_OR(DT_NODELABEL(aclk), clock_frequency, \
                                    DT_PROP_OR(DT_NODELABEL(clock), hfclkaudio_frequency, 0))
#endif

static void free_buffer(struct t5838_drv_data *drv_data, void *buffer)
{
	k_mem_slab_free(drv_data->mem_slab, buffer);
	LOG_DBG("Freed buffer %p", buffer);
}

static void stop_pdm(struct t5838_drv_data *drv_data)
{
	drv_data->stopping = true;
	nrfx_pdm_stop(&drv_data->pdm);
}

static int request_clock(struct t5838_drv_data *drv_data)
{
	if (!drv_data->request_clock) {
		return 0;
	}
#if CONFIG_CLOCK_CONTROL_NRFS_AUDIOPLL || DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
	return nrf_clock_control_request(drv_data->audiopll_dev, NULL, &drv_data->clk_cli);
#elif CONFIG_CLOCK_CONTROL_NRF
	return onoff_request(drv_data->clk_mgr, &drv_data->clk_cli);
#else
	return -ENOTSUP;
#endif
}

static int release_clock(struct t5838_drv_data *drv_data)
{
	if (!drv_data->request_clock) {
		return 0;
	}
#if CONFIG_CLOCK_CONTROL_NRFS_AUDIOPLL || DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
	return nrf_clock_control_release(drv_data->audiopll_dev, NULL);
#elif CONFIG_CLOCK_CONTROL_NRF
	return onoff_release(drv_data->clk_mgr);
#else
	return -ENOTSUP;
#endif
}

static void event_handler(const struct device *dev, const nrfx_pdm_evt_t *evt)
{
	struct t5838_drv_data *drv_data = dev->data;
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	int ret;
	bool stop = false;
	void *mem_slab_buffer;

	if (evt->buffer_requested) {
		void *buffer;
		int err;

		ret = k_mem_slab_alloc(drv_data->mem_slab, &mem_slab_buffer, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Failed to allocate buffer: %d", ret);
			stop = true;
		} else {
			ret = dmm_buffer_in_prepare(drv_cfg->mem_reg, mem_slab_buffer,
						    drv_data->block_size, &buffer);
			if (ret < 0) {
				LOG_ERR("Failed to prepare buffer: %d", ret);
				free_buffer(drv_data, mem_slab_buffer);
				stop_pdm(drv_data);
				return;
			}
			ret = k_msgq_put(&drv_data->mem_slab_queue, &mem_slab_buffer, K_NO_WAIT);
			if (ret < 0) {
				LOG_ERR("Unable to put mem slab in queue");
				free_buffer(drv_data, mem_slab_buffer);
				stop_pdm(drv_data);
				return;
			}
			err = nrfx_pdm_buffer_set(&drv_data->pdm, buffer, drv_data->block_size / 2);
			if (err != 0) {
				LOG_ERR("Failed to set buffer: 0x%08x", err);
				stop = true;
			}
		}
	}

	if (drv_data->stopping) {
		if (evt->buffer_released) {
			ret = k_msgq_get(&drv_data->mem_slab_queue, &mem_slab_buffer, K_NO_WAIT);
			if (ret < 0) {
				LOG_ERR("No buffers to free");
				return;
			}
			ret = dmm_buffer_in_release(drv_cfg->mem_reg, mem_slab_buffer,
						    drv_data->block_size, evt->buffer_released);
			if (ret < 0) {
				LOG_ERR("Failed to release buffer: %d", ret);
				free_buffer(drv_data, mem_slab_buffer);
				return;
			}
			free_buffer(drv_data, mem_slab_buffer);
		}

		if (drv_data->active) {
			drv_data->active = false;
			ret = release_clock(drv_data);
			if (ret < 0) {
				LOG_ERR("Failed to release clock: %d", ret);
				return;
			}
		}
	} else if (evt->buffer_released) {
		ret = k_msgq_get(&drv_data->mem_slab_queue, &mem_slab_buffer, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("No buffers to free");
			stop_pdm(drv_data);
			return;
		}
		ret = dmm_buffer_in_release(drv_cfg->mem_reg, mem_slab_buffer,
					    drv_data->block_size, evt->buffer_released);
		if (ret < 0) {
			LOG_ERR("Failed to release buffer: %d", ret);
			free_buffer(drv_data, mem_slab_buffer);
			stop_pdm(drv_data);
			return;
		}

		ret = k_msgq_put(&drv_data->rx_queue, &mem_slab_buffer, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("No room in RX queue");
			stop = true;
			free_buffer(drv_data, mem_slab_buffer);
		} else {
			LOG_DBG("Queued buffer %p", evt->buffer_released);
		}
	}

	if (stop) {
		stop_pdm(drv_data);
	}
}

static int t5838_configure(const struct device *dev, struct dmic_cfg *config)
{
	struct t5838_drv_data *drv_data = dev->data;
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	struct pdm_chan_cfg *channel = &config->channel;
	struct pcm_stream_cfg *stream = &config->streams[0];
	uint32_t def_map, alt_map;
	nrfx_pdm_config_t nrfx_cfg;
	int err;

	if (drv_data->active) {
		LOG_ERR("Cannot configure device while it is active");
		return -EBUSY;
	}

	/*
	 * This device supports only one stream and can be configured to return
	 * 16-bit samples for two channels (Left+Right samples) or one channel
	 * (only Left samples). Left and Right samples can be optionally swapped
	 * by changing the PDM_CLK edge on which the sampling is done
	 * Provide the valid channel maps for both the above configurations
	 * (to inform the requester what is available) and check if what is
	 * requested can be actually configured.
	 */
	if (channel->req_num_chan == 1) {
		def_map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
		alt_map = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT);

		channel->act_num_chan = 1;
	} else {
		def_map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
			  dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
		alt_map = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT) |
			  dmic_build_channel_map(1, 0, PDM_CHAN_LEFT);

		channel->act_num_chan = 2;
	}

	channel->act_num_streams = 1;
	channel->act_chan_map_hi = 0;

	if (channel->req_num_streams != 1 ||
	    channel->req_num_chan > 2 ||
	    channel->req_num_chan < 1 ||
	    (channel->req_chan_map_lo != def_map &&
	     channel->req_chan_map_lo != alt_map) ||
	    channel->req_chan_map_hi != channel->act_chan_map_hi) {
		LOG_ERR("Requested configuration is not supported");
		return -EINVAL;
	}

	/* If either rate or width is 0, the stream is to be disabled. */
	if (stream->pcm_rate == 0 || stream->pcm_width == 0) {
		if (drv_data->configured) {
			nrfx_pdm_uninit(&drv_data->pdm);
			drv_data->configured = false;
		}

		return 0;
	}

	if (stream->pcm_width != 16) {
		LOG_ERR("Only 16-bit samples are supported");
		return -EINVAL;
	}

	nrfx_cfg = drv_cfg->nrfx_def_cfg;
	nrfx_cfg.mode = channel->req_num_chan == 1
		      ? NRF_PDM_MODE_MONO
		      : NRF_PDM_MODE_STEREO;
	if (channel->req_chan_map_lo == def_map) {
		nrfx_cfg.edge = NRF_PDM_EDGE_LEFTFALLING;
		channel->act_chan_map_lo = def_map;
	} else {
		nrfx_cfg.edge = NRF_PDM_EDGE_LEFTRISING;
		channel->act_chan_map_lo = alt_map;
	}
#if NRF_PDM_HAS_SELECTABLE_CLOCK
	nrfx_cfg.mclksrc = drv_cfg->clk_src == ACLK
		         ? NRF_PDM_MCLKSRC_ACLK
			 : NRF_PDM_MCLKSRC_PCLK32M;
#endif
        nrfx_pdm_output_t output_config = {
                .base_clock_freq = (NRF_PDM_HAS_SELECTABLE_CLOCK && drv_cfg->clk_src == ACLK)
                                        ? DMIC_T5838_AUDIO_CLOCK_FREQ
                                        : DMIC_T5838_CLOCK_FREQ,
                .sampling_rate = config->streams[0].pcm_rate,
                .output_freq_min = config->io.min_pdm_clk_freq,
                .output_freq_max = config->io.max_pdm_clk_freq
        };

        if (nrfx_pdm_prescalers_calc(&output_config, &nrfx_cfg.prescalers) != 0) {
                LOG_ERR("Cannot find suitable PDM clock configuration.");
                return -EINVAL;
        }

	if (drv_data->configured) {
		nrfx_pdm_uninit(&drv_data->pdm);
		drv_data->configured = false;
	}

	err = nrfx_pdm_init(&drv_data->pdm, &nrfx_cfg, drv_cfg->event_handler);
	if (err != 0) {
		LOG_ERR("Failed to initialize PDM: 0x%08x", err);
		return -EIO;
	}

	drv_data->block_size = stream->block_size;
	drv_data->mem_slab = stream->mem_slab;

	/* Unless the PCLK32M source is used with the HFINT oscillator
	 * (which is always available without any additional actions),
	 * it is required to request the proper clock to be running
	 * before starting the transfer itself.
         * Targets using CLKSELECT register to select clock source
         * do not need to request audio clock.
	 */
	drv_data->request_clock = (drv_cfg->clk_src != PCLK32M && !NRF_PDM_HAS_CLKSELECT);
	drv_data->configured = true;
	return 0;
}

static int start_transfer(struct t5838_drv_data *drv_data)
{
	int err;
	int ret;

	err = nrfx_pdm_start(&drv_data->pdm);
	if (err == 0) {
		return 0;
	}

	LOG_ERR("Failed to start PDM: %d", err);
	ret = -EIO;

	ret = release_clock(drv_data);
	if (ret < 0) {
		LOG_ERR("Failed to release clock: %d", ret);
		return ret;
	}

	drv_data->active = false;
	return ret;
}

static void clock_started_callback(struct onoff_manager *mgr, struct onoff_client *cli,
				   uint32_t state, int res)
{
	struct t5838_drv_data *drv_data = CONTAINER_OF(cli, struct t5838_drv_data, clk_cli);

	/* The driver can turn out to be inactive at this point if the STOP
	 * command was triggered before the clock has started. Do not start
	 * the actual transfer in such case.
	 */
	if (!drv_data->active) {
		int ret = release_clock(drv_data);

		if (ret < 0) {
			LOG_ERR("Failed to release clock: %d", ret);
			return;
		}
	} else {
		(void)start_transfer(drv_data);
	}
}

static int trigger_start(const struct device *dev)
{
	struct t5838_drv_data *drv_data = dev->data;
	int ret;

	drv_data->active = true;

	/* If it is required to use certain HF clock, request it to be running
	 * first. If not, start the transfer directly.
	 */
	if (drv_data->request_clock) {
		sys_notify_init_callback(&drv_data->clk_cli.notify,
			                 clock_started_callback);
		ret = request_clock(drv_data);
		if (ret < 0) {
			drv_data->active = false;

			LOG_ERR("Failed to request clock: %d", ret);
			return -EIO;
		}
	} else {
		ret = start_transfer(drv_data);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int t5838_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct t5838_drv_data *drv_data = dev->data;

	switch (cmd) {
	case DMIC_TRIGGER_PAUSE:
	case DMIC_TRIGGER_STOP:
		if (drv_data->active) {
			drv_data->stopping = true;
			nrfx_pdm_stop(&drv_data->pdm);
		}

#ifdef CONFIG_T5838_AAD_TRIGGER
		if (drv_data->aad_child_dev != NULL) {
			t5838_aad_sleep(drv_data->aad_child_dev);
		}
#endif
		break;

	case DMIC_TRIGGER_RELEASE:
	case DMIC_TRIGGER_START:
		if (!drv_data->configured) {
			LOG_ERR("Device is not configured");
			return -EIO;
		} else if (!drv_data->active) {
			drv_data->stopping = false;
			return trigger_start(dev);
		}
		break;

	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int t5838_pdm_read(const struct device *dev, uint8_t stream, void **buffer, size_t *size,
			  int32_t timeout)
{
	struct t5838_drv_data *drv_data = dev->data;
	int ret;

	ARG_UNUSED(stream);

	if (!drv_data->configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}

	ret = k_msgq_get(&drv_data->rx_queue, buffer, SYS_TIMEOUT_MS(timeout));
	if (ret != 0) {
		LOG_ERR("No audio data to be read");
	} else {
		LOG_DBG("Released buffer %p", *buffer);

		*size = drv_data->block_size;
	}

	return ret;
}

static void init_clock_manager(const struct device *dev)
{
#if DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
	struct t5838_drv_data *drv_data = dev->data;
	drv_data->audiopll_dev = DEVICE_DT_GET(NODE_AUDIO_AUXPLL);
#elif CONFIG_CLOCK_CONTROL_NRF
	clock_control_subsys_t subsys;
	struct t5838_drv_data *drv_data = dev->data;
#if NRF_CLOCK_HAS_HFCLKAUDIO
	const struct t5838_drv_cfg *drv_cfg = dev->config;

	if (drv_cfg->clk_src == ACLK) {
		subsys = CLOCK_CONTROL_NRF_SUBSYS_HFAUDIO;
	} else
#endif
	{
		subsys = CLOCK_CONTROL_NRF_SUBSYS_HF;
	}

	drv_data->clk_mgr = z_nrf_clock_control_get_onoff(subsys);
	__ASSERT_NO_MSG(drv_data->clk_mgr != NULL);
#elif CONFIG_CLOCK_CONTROL_NRFS_AUDIOPLL
	struct t5838_drv_data *drv_data = dev->data;
	drv_data->audiopll_dev = DEVICE_DT_GET(NODE_AUDIOPLL);
#endif
}

static const struct _dmic_ops t5838_dmic_ops = {
	.configure = t5838_configure,
	.trigger = t5838_trigger,
	.read = t5838_pdm_read,
};

#define T5838_PDM_CLK_SRC(inst) DT_STRING_TOKEN(DT_DRV_INST(inst), clock_source)

#define T5838_PDM_NRFX_DEVICE(inst)                                                                \
	static void *t5838_rx_msgs##inst[DT_INST_PROP(inst, queue_size)];                          \
	static void *mem_slab_msgs##inst[DT_INST_PROP(inst, queue_size)];                          \
	static struct t5838_drv_data t5838_data##inst = {                                          \
		.pdm = NRFX_PDM_INSTANCE(DT_INST_REG_ADDR(inst)),                                  \
	};                                                                                         \
	static int t5838_pdm_nrfx_init##inst(const struct device *dev)                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), nrfx_pdm_irq_handler, \
			    &t5838_data##inst.pdm, 0);                                             \
		const struct t5838_drv_cfg *drv_cfg = dev->config;                                 \
		int err = pinctrl_apply_state(drv_cfg->pcfg, PINCTRL_STATE_DEFAULT);               \
		if (err < 0) {                                                                     \
			return err;                                                                \
		}                                                                                  \
		k_msgq_init(&t5838_data##inst.rx_queue, (char *)t5838_rx_msgs##inst,               \
		            sizeof(void *), ARRAY_SIZE(t5838_rx_msgs##inst));                      \
		k_msgq_init(&t5838_data##inst.mem_slab_queue, (char *)mem_slab_msgs##inst,         \
	                    sizeof(void *), ARRAY_SIZE(mem_slab_msgs##inst));                      \
		init_clock_manager(dev);                                                           \
		return 0;                                                                          \
	}                                                                                          \
	static void event_handler##inst(const nrfx_pdm_evt_t *evt)                                 \
	{                                                                                          \
		event_handler(DEVICE_DT_INST_GET(inst), evt);                                      \
	}                                                                                          \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct t5838_drv_cfg t5838_cfg##inst = {                                       \
		.event_handler = event_handler##inst,                                              \
		.nrfx_def_cfg = NRFX_PDM_DEFAULT_CONFIG(0, 0),                                     \
		.nrfx_def_cfg.skip_gpio_cfg = true,                                                \
		.nrfx_def_cfg.skip_psel_cfg = true,                                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.clk_src = T5838_PDM_CLK_SRC(inst),                                                \
		.mem_reg = DMM_DEV_TO_REG(DT_DRV_INST(inst)),                                      \
	};                                                                                         \
	NRF_DT_CHECK_NODE_HAS_REQUIRED_MEMORY_REGIONS(DT_DRV_INST(inst));                          \
	BUILD_ASSERT(T5838_PDM_CLK_SRC(inst) != ACLK || NRF_PDM_HAS_SELECTABLE_CLOCK,              \
		     "Clock source ACLK is not available.");                                       \
	BUILD_ASSERT(T5838_PDM_CLK_SRC(inst) != ACLK ||                                            \
			     DT_NODE_HAS_PROP(DT_NODELABEL(clock), hfclkaudio_frequency) ||        \
			     DT_NODE_HAS_PROP(DT_NODELABEL(aclk), clock_frequency) ||              \
			     DT_NODE_HAS_PROP(NODE_AUDIOPLL, frequency) ||                         \
			     DT_NODE_HAS_PROP(NODE_AUDIO_AUXPLL, nordic_frequency),                \
                     "Clock source ACLK requires one following defined frequency "                 \
                     "properties: "                                                                \
                     "hfclkaudio-frequency in the nordic,nrf-clock node, "                         \
                     "clock-frequency in the aclk node, "                                          \
                     "frequency in the audiopll node, "                                            \
                     "nordic-frequency in the audio_auxpll node");                                 \
	DEVICE_DT_INST_DEFINE(inst, t5838_pdm_nrfx_init##inst, NULL, &t5838_data##inst,                 \
			      &t5838_cfg##inst, POST_KERNEL,                                            \
			      CONFIG_AUDIO_DMIC_INIT_PRIORITY, &t5838_dmic_ops);

DT_INST_FOREACH_STATUS_OKAY(T5838_PDM_NRFX_DEVICE)
