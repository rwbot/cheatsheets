# How to Configure Custom Events for Mouse Buttons

**These button numbers correspond to the Logitech M510 Wireless Mouse on Ubuntu 18.04.2**

### **1. Install tools**
```
sudo apt-get install xbindkeys xautomation x11-utils
```
### **2. Identify ID of desired buttons**
`xev` dumps any and all events from the display, so isolate only mouse button events
```
xev -event mouse | grep Button --before-context=1 --after-context=2
```
### **3. Create `xbindkeys` config file**
```
xbindkeys --defaults > $HOME/.xbindkeysrc  
```
### **4. Add desired commands** 
Open the config file
```
gedit $HOME/.xbindkeysrc
```
Add the combinations of buttons and keys for the functionality you want, ex:
```bash
# Scroll Wheel Down Click = CTRL+W = Close TAB 
"xte 'keydown Control_L' 'key w' 'keyup Control_L'"
    b:2 + Release

# Scroll Wheel Click Left = CTRL+Plus = ZOOM IN
"xte 'keydown Control_L' 'key plus' 'keyup Control_L'"
    b:6 + Release

# Scroll Wheel Click Right = CTRL+Minus = ZOOM OUT
"xte 'keydown Control_L' 'key minus' 'keyup Control_L'"
    b:7 + Release

# Thumb Back = CTRL+ALT+T = Open Terminal
"xte 'keydown Control_L' 'keydown Alt_L' 'key t' 'keyup Control_L' 'keyup Alt_L'"
    b:8 + Release
    
# Thumb Front = CTRL+Left Click = Open in New Tab
"xte 'keydown Control_L' 'mouseclick 1' 'keyup Control_L'"
    b:9 + Release
```
### **5. Apply changes**
```
xbindkeys -p
```
---
Source: [https://askubuntu.com/questions/152297/how-to-configure-extra-buttons-in-logitech-mouse](https://askubuntu.com/questions/152297/how-to-configure-extra-buttons-in-logitech-mouse)
