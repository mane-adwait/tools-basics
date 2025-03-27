Thursday 2025 March 27

--- Recreate the virtual environment in the root of tools-basics.
My virtual environment directory (.ve) is outside the tools-basics repo. It is currently located at ~/Code. Best practice is to save it in the root of the repo i.e. ~/Code/tools-basics.
Moving .ve can lead to issues since virtual environment scripts store absolute paths.