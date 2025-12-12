# RobotVision

## Prerequisites

Install and configure conan:

* Make sure `python3`, `pip`, and `virtualenv` are available
* Create a virtual env:

  ```
  python3 -m venv .venv
  ```

* Activate virtual env:

  ```
  source .venv/bin/activate
  ```

* Install [conan](https://docs.conan.io/2/index.html):
  
  ```
  python -m pip install conan
  ```

* Configure conan:

  ```
  conan profile detect --force
  ```

## Install Dependencies

This step ensures the dependencies are available. Any time `conanfile.txt` is changed, the
dependencies will need to be updated with the command below (example for Ubuntu):

```
conan install . \
  --output-folder=build \
  --build=missing \
  -c tools.system.package_manager:mode=install \
  -c tools.system.package_manager:sudo=True \
  -c tools.system.package_manager:tool=apt-get
```

## Build the Project

In the top repository directory:

* Generate makefiles using conan toolchain:

  ```
  cmake -B build --preset conan-release
  ```

* Run the build:

  ```
  cmake --build --preset conan-release
  ```