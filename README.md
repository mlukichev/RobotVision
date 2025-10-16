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
dependencies will need to be updated with the command below:

```
conan install . --output-folder=build --build=missing
```

## Build the Project

Change to the `build` directory:

```
cd build
```

Generate makefiles using conan toolchain:

```
cmake .. --preset conan-release
```

Run the build:

```
make
```