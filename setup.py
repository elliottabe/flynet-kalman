from skbuild import setup

setup(
        name = "flynet-kalman",
        version = "0.1.0",
        description = "kalman filter for flynet",
        author = "Johan Melis",
        license = "MIT",
        python_requires = ">=3.8",
        packages=["flynet_kalman"],
        )
