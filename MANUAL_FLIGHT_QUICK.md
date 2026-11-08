# Manual flight: quick steps

In **Git Bash**, from the repo folder, on the drone's Wi-Fi. Have the drone password ready.

1. **Get the code:** `git checkout main && git pull`
2. **Put it on the drone:** `tools/manual-flight setup "<your name>"`
3. **Power-cycle:** battery out 10 s, back in, wait **3 minutes**.
4. **Check:** `tools/manual-flight check` → must say **GO**. NO-GO means don't fly: power-cycle and check again.
5. **Fly** by hand. Kill switch in hand.
6. **Land, disarm, leave the battery in.**
7. **Get the data:** `tools/manual-flight pull` → saved in `flight-logs/<date-time>/`
8. *(Motor outputs, full-rate sticks)* USB cable to the Pixhawk → QGroundControl → **Analyze Tools → Log Download** → newest.

**Next flight:** swap the battery, wait 3 minutes, go back to step 4.
**Done for the day:** `tools/manual-flight restore`, then power-cycle.

Full details and troubleshooting: `MANUAL_FLIGHT.md`.
