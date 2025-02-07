from setuptools import setup

# Define the package name.
package_name = 'path_finding_visualizer'

setup(
    # The name of the package.
    name=package_name,
    # Package version.
    version='0.0.1',
    # List all packages that should be included in the distribution.
    # Here, we assume that your Python modules are contained within a folder named `path_finding_visualizer`.
    packages=[package_name],
    # Data files to be included, such as package manifest files.
    data_files=[
        # This will install the package.xml file into the share directory for your package.
        ('share/' + package_name, ['package.xml']),
    ],
    # Specify the packages required to run this package.
    install_requires=['setuptools'],
    # Whether the package can be safely installed as a .zip file.
    zip_safe=True,
    # Package author name.
    author='Your Name',
    # Package author email.
    author_email='your.email@example.com',
    # Short description of the package.
    description='A ROS2 package that visually demonstrates 10 path-finding algorithms on a 20x20 grid with obstacles.',
    # License type for the package.
    license='Apache License 2.0',
    # Additional packages required for running tests.
    tests_require=['pytest'],
    # Define entry points for the console scripts.
    # This allows you to run each algorithm's node directly from the command line.
    entry_points={
        'console_scripts': [
            # Each entry is in the form "command = module:function"
            # When you run, for example, `ros2 run path_finding_visualizer dijkstra`,
            # it will execute the main() function inside the dijkstra.py module.
            'dijkstra = path_finding_visualizer.dijkstra:main',
            'astar = path_finding_visualizer.astar:main',
            'bfs = path_finding_visualizer.bfs:main',
            'dfs = path_finding_visualizer.dfs:main',
            'greedy_bfs = path_finding_visualizer.greedy_bfs:main',
            'uniform_cost = path_finding_visualizer.uniform_cost:main',
            'bidirectional = path_finding_visualizer.bidirectional:main',
            'jump_point_search = path_finding_visualizer.jump_point_search:main',
            'rbfs = path_finding_visualizer.recursive_best_first_search:main',
            'fringe_search = path_finding_visualizer.fringe_search:main',
        ],
    },
)
