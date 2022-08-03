# BlenderTekkenDirect

Preview animations in Tekken 7 live from blender. Supports live body animation, live face animation and live hand animations (check the Armature layers for each animatable part)
You need to be working on one the specific models provided in the following link for it to work: https://mega.nz/folder/05pBGLAL#XPHcsgK2tBxT3RLMbZPaFw

![Pose comparison preview](https://i.imgur.com/5v6e0Xi.jpg)

# Installation
(I did my testing on Blender 2.92.0, its python version is 3.7.7)

1. Download the project as a .zip.
2. Rename the .zip precisely as **BlenderTekkenDirect.zip**
- Manual: Extract the archive so that it creates a folder here: **AppData\Roaming\Blender Foundation\Blender\VERSION\scripts\addons**. Make sure the folder is correctly named **BlenderTekkenDirect** 
- Automatic: Open Blender,go to **Edit -> Preferences**, in the newly opened window go to the **Addons** tab, click **Install** and select the .zip you downloaded earlier. If you get a 'No module named BlenderTekkenDirect' error, make sure the BlenderTekkenDirect directory in **AppData\Roaming\Blender Foundation\Blender\VERSION\scripts\addons** does not have a different name than **BlenderTekkenDirect**
3. An new tab should now appear at the corner of the 3d viewport
4. If the addon is still not appearing, make sure it is enabled in **Edit -> Preferences -> Add-ons**

![3d viewport top-right corner](https://i.imgur.com/8jq9tGN.png)

I also had to manually install pywin32 for this to work.
1. Locate your Blender installation and execute the following command from cmd.exe : `Blender\2.92\python\bin\python.exe -m pip install pywin32`
2. Execute the same command again so that the program tells you where the downloaded folders are located (for me, it was in `%appdata%\python\python37\site-packages`)
3. Open the explorer and go to the path obtained above. Copy the following folders and paste them in blender's site-packages folder (located at `Blender\2.92\python\lib\site-packages`)
- win32
- win32com
- win32comext
- pywin32_system32

4. Pywin32 should now be installed on blender
