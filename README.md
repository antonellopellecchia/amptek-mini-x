# Amptek Mini-X
Library to control the Amptek Mini-X using the pyftdi library. The library is not tested
thoroughly, so use at your own risk!

Note: The following udev-rule is necessary (copy to `/etc/udev/rules.d/20-amptek.rules`):
```
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="d058", GROUP="users", MODE="0660", SYMLINK+="amptek/minix%n"
```
