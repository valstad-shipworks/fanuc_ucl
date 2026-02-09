from fanuc_ucl import hmi

def main():
    driver = hmi.HmiDriver("10.0.0.1")
    driver.connect()

    if driver.read(hmi.DigitalOutput, 1).wait_timeout(0.01):
        print("DO1 is on")
    else:
        print("DO1 is off")

    groups = driver.read(hmi.GroupInput, 1, 40).wait_timeout(0.01)
    for i, group in enumerate(groups):
        print(f"Group {i + 1}: {group}")

    driver.write(hmi.DigitalOutput, 1, True)
    driver.write(hmi.GroupOutput, 1, [10, 11, 12, 13])
    driver.write_unsafe(hmi.GroupInput, 1, [10, 11, 12, 13])

    hspo_enable_var = driver.register_sysvar_asg(bool, name="$MCGR_CFG.$ENABLE")
    if hspo_enable_var.read(driver).wait_timeout(0.01):
        print("HSPO is already enabled")
    else:
        hspo_enable_var.write(driver, True)
        print("HSPO is now enabled")

    # ASG has ALOT of functionality, so we won't cover it all here.
    # When the Rustdocs are finished it will delve more into it.