from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mycobot_opcua_control'

setup(
    name=package_name,
    version='0.0.0', # package.xml의 버전과 일치시키세요.
    
    # 필수 추가 항목: 패키지 디렉토리를 찾아 등록
    packages=find_packages(exclude=['test']), 
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 경로 추가
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leewonyoung', # package.xml의 maintainer와 일치시키세요.
    maintainer_email='leewonyoung@todo.todo', # package.xml의 email과 일치시키세요.
    description='MyCobot control via OPC UA PLC trigger', # 적절한 설명
    license='TODO: License declaration', # package.xml의 라이선스와 일치시키세요.
    tests_require=['pytest'],
    
    # 필수 항목: 실행 가능한 노드 파일을 등록
    entry_points={
        'console_scripts': [
            # '실행할 명령어 = 모듈 이름.파일 이름(확장자 제외):함수 이름' 
            'OPCUA_Server = mycobot_opcua_control.OPCUA_Server:main',
            'M0001 = mycobot_opcua_control.M0001:main',
            'M0010 = mycobot_opcua_control.M0010:main'
        ],
    },
)