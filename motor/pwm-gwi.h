#ifndef ___PWM_GWI_H__
#define ___PWM_GWI_H__

extern int imx_pwm_set_callback(struct pwm_device *pwm,  irq_handler_t handler, void * dev_id);

#endif