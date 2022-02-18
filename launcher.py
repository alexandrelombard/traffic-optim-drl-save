import environment_class


environment = environment_class.Environment(mode=2, display=True, training=False, simulation_time=1000, image_size=50,
                                           reward_type="mix_vehicle_time", coef=10, flow1=700,
                                           flow2=700, flow3=700, flow4=700)
environment.launch_simulation()
