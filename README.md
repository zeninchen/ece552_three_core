## ⚙️ How to Run / Simulate

This project's RTL was compiled and verified using **Siemens ModelSim**. To run the testbenches:

1. Launch ModelSim and create a new project in the cloned directory.
2. Add all the `.v` files (and `program.mem`) from the `rtl` and `tb` folders (inside `project6-and-7`), and compile them.
3. Load the top-level testbench (`stall_mem_tb.v`).
4. Add the desired signals to the wave window and run the simulation to view the datapath execution.
5. Check the ModelSim transcript window, which outputs the execution results.
6. **Custom Execution:** You can modify the `program.mem` file inside the `rtl` folder to load and run your own desired instructions through the pipeline.
