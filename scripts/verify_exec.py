import sys
import os
from robot_interface.src.robot_client_interface import execute_task_program

# Simulating a user program that just prints something
# Important: this defines task_program() because that's what the system assumes
program = """
def task_program():
    print("Hello from subprocess!")
    if 'go_to' in locals() or 'go_to' in globals():
        print("go_to is available")
    else:
        print("go_to is MISSING")
"""

# We don't need a real robot instance for this test, as the subprocess will create a new one.
# But execute_task_program expects one.
class MockRobot:
    pass

print("Running verification...")
try:
    execute_task_program(program, MockRobot())
    print("Verification passed (no crash).")
except Exception as e:
    print(f"Verification failed: {e}")
