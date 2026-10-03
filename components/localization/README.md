# localization

State estimation for each drone and the payload. Empty for now: the container
exists so it is already part of the ROS graph when the estimators land.

Raw UWB ranges come from the vehicle component, which owns the UWB hardware and
its firmware ([`../vehicle/uwb_firmware`](../vehicle/uwb_firmware)); this
component turns them into position estimates.
