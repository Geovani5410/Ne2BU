from setuptools import find_packages, setup

package_name = 'main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='geovani',
    maintainer_email='geovani@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'prueba = main.prueba:main',
            'activate =main.activate:main',
            'wake_word = main.wake_word:main',
            'voz_node = main.voz_node:main',
            'llm_node = main.llm_node:main',
            'tts_node =main.tts_node:main',
            'history_node = main.history_node:main',
            'bridge_node = main.bridge_node:main',
            'dashboard = main.dashboard:main',
            'emotion = main.emotion:main'
        ],
    },
)
