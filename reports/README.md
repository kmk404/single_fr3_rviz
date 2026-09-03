# Test artifacts

Raw logs are generated locally and intentionally not committed:

- `docker_build.log`: Jazzy/Noble image build output
- `colcon_build.log`: workspace build output
- `colcon_test.log`: ament/colcon test output
- `fake_mode.log`: headless fake-mode launch and trajectory Action smoke test
- `real_no_ip.log`: required early-failure test for real mode without an IP

The checked-in evidence is `test_report.md` and the verified
`rviz_fake.png` screenshot.
