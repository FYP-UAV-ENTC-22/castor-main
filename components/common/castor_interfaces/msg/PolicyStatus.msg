# The planning component's policy runner, on <ns>/planning/status (1 Hz).

std_msgs/Header header
uint32 team_size
uint32 team_index
float32[] one_hot            # agent id vector built from robot.yaml
bool model_loaded
bool obs_dim_ok              # model input width == (obs_frame_base + team_size) * history
uint32 expected_obs_dim
uint32 model_obs_dim         # 0 if no model
float64 loop_rate_hz         # measured over the last second
float64 max_jitter_ms        # worst timer lateness over the last second
float64 last_inference_ms    # -1 if no model
