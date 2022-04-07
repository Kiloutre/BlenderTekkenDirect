# BlenderTekkenDirect

Preview animations in Tekken 7 live from blender. Eventually will be able to save them.
You need to be working on the specific provided model in the following link for things to work: https://mega.nz/folder/05pBGLAL#XPHcsgK2tBxT3RLMbZPaFw

![Pose comparison preview](https://i.imgur.com/5v6e0Xi.jpg)

# Blender problems

I did my testing on Blender 2.92.0, its python version is 3.7.7

I also had to manually install pywin32 inside `Blender\2.92\python\lib\site-packages` by copying the following folders to it (obtained from another python install i had where i executed the `python -m pip install pywin32 --user` command):
- win32
- win32com
- win32comext
- pywin32_system32
