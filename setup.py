from setuptools import setup, find_packages
# Read in the requirements.txt file
with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(
    name='codebotler',
    version='0.1',
    packages=find_packages(['roboeval', 'codebotler_robot_interface']),
    install_requires=requirements,
    author='Zichao Hu',
    author_email='zichao@utexas.edu',
    description='Codebotler RoboEval Implementation',
    url='https://github.com/ut-amrl/codebotler',
    python_requires='>=3.10',
)