# Line_Navigation_Offline

Offline version of the code for the paper  
Bista SR, Giordano PR, Chaumette F. Appearance-based indoor navigation by IBVS using line segments. IEEE Robotics and Automation Letters. 2016 Jan 26;1(1):423-30.

The code is open source. The codes are provided “as-is” without any warranty. Before using the code, you agree to use the code at
your own risk. The authors are not responsible or liable for any damages incurred using this code.


# Build Instructions

### 1. Get Source codes from the repository 
   
```
git clone <reopo url>
cd line_navigation_offline
git submodule update --init selectKeyImagesLines
```

### 2. Install required dependencies

```
sudo apt install libarpack2-dev libsuperlu-dev
```
	
### 3. Build executables required for mapping.

``` 
cd selectKeyImagesLines  
./build_linematching.sh
```

### 4. Build Navigation code
    
For first time use setup.sh to compile the code. This will build custom ARPACK and SUPELU libaries

```
cd ..
./setup.sh 
```

After initial setup, the further compilation can be done by  

```
./compile.sh
```

### 5. Steps to run naviagtion code after building
     a) Get Reference images from teach sequence. (Refer https://github.com/suuman/selectKeyImagesLines)
     b) Test Script to run code `run.sh`
        
      



