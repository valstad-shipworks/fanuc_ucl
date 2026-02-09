use std::time::Duration;

use fanuc_ucl::hmi::{DigitalOutput, GroupInput, GroupOutput, HmiDriver, SysVarArgs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut driver = HmiDriver::new([10, 0, 0, 1]);
    driver.connect(Some(Duration::from_secs(1)), None)?;

    if driver.read::<DigitalOutput>(1)?.wait_timeout(Duration::from_millis(10))? {
        println!("DO1 is on");
    } else {
        println!("DO1 is off");
    }

    let groups = driver.read_array::<GroupInput>(1, 40)?.wait_timeout(Duration::from_millis(10))?;
    for (i, group) in groups.iter().enumerate() {
        println!("Group {}: {:?}", i + 1, group);
    }

    driver.write::<DigitalOutput>(1, true)?;
    driver.write_array::<GroupOutput>(1, &[10, 11, 12, 13])?;
    driver.write_array_unsafe::<GroupInput>(1, &[10, 11, 12, 13])?;

    let hspo_enable_var = driver.register_asg(
        SysVarArgs{
            var_name: "$MCGR_CFG.$ENABLE".to_string(),
            ..Default::default()
        },
        Duration::from_millis(24)
    )?;
    if hspo_enable_var.read(&driver)?.wait_timeout(Duration::from_millis(10))? {
        println!("HSPO is already enabled");
    } else {
        hspo_enable_var.write(&driver, true)?;
        println!("HSPO is now enabled");
    }

    // ASG has ALOT of functionality, so we won't cover it all here.
    // When the Rustdocs are finished it will delve more into it.

    Ok(())
}