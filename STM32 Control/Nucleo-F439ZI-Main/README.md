# GoKart Real Time Controller

Running on the **[NUCLEO-F439ZI]** board with the **[STM32F439ZIT6]** MCU.

Written in C. Using STM32CubeF7 HAL (see [GitHub repo](https://github.com/STMicroelectronics/stm32f4xx_hal_driver),
see [product page with docs on st.com](https://www.st.com/en/embedded-software/stm32cubef4.html#documentation),
see [UM1725 Description of STM32F4 HAL and low-layer drivers][UM1725]).


## STM32CubeMX

**Note:** _This section is here only for future reference. You don't need to download STM32CubeMX and don't need to
follow steps in this section._

This project was created by [STM32CubeMX]. Here is the procedure we used:
1. _New Project_ > _Board Selector_ > **NUCLEO-F439ZI** > _Start Project_ > _Initialize all peripherals with their
   default Mode?_
   **Yes**
2. Then in the _Project Manager_ tab:
	1. Fill in the _Project Name._
	2. Change the _Application structure_ to **Basic**. Keep the _Do not generate the main()_ unchecked.
	3. Change the _Toolchain / IDE_ to STM32CubeIDE (so that the project is compatible with CLion). **Check** _Generate
	   Under Root_ option.
	4. The other fields should be okay with the default values.


## Development

**Use JetBrains [CLion] (free for non-commercial use for students) for development.**
The project is already imported and fully configured, use _File > Open..._ to just open it.

**But before** opening, you'll probably need to install a few things in your system:
1. [ARM GNU Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/downloads)
2. [OpenOCD](https://openocd.org/pages/getting-openocd.html)

If you have all the tools installed, you should be able

You can read more in this [CLion's Embedded development with STM32CubeMX projects][CLion-Embedded-Development]
guide.


## SVD file for the MCU

CLion and other IDEs support SVD files for describing the layout of registers for debugging.

**Note:** We downloaded the SVD file to [svd/STM32F439x.svd](./svd/STM32F439x.svd) so you don't need to download it
yourselves.

ST does not provide SVD files for their MCUs. Fortunately, we can get one from the ARM Keil IDE MDK5 pack.
1. See [MDK5 Software Packs](https://www.keil.com/dd2/pack/) and look for _STMicroelectronics STM32F4 Series Device
   Support_. Here is a [direct download link](https://keilpack.azureedge.net/pack/Keil.STM32F4xx_DFP.2.16.0.pack) (
   might stop working in the future).
2. Download the pack (Note: At the time we downloaded it, it
   was `Version: 2.16.0 (2022-01-19) Keil.STM32F4xx_DFP.2.16.0.pack`.). and Examples_.
3. Rename `.pack` to `.zip`. Then unzip it.
4. Open the `Keil.STM32F4xx_DFP.2.16.0` dir, go to `CMSIS/SVD` subdir. There should be the `STM32F439x.svd` file. That's
   the one you are looking for.


<!-- links references -->

[NUCLEO-F439ZI]: https://www.st.com/en/evaluation-tools/nucleo-f439zi.html

[STM32F439ZIT6]: https://www.st.com/en/microcontrollers-microprocessors/stm32f439zi.html

[UM1725]: https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf

[STM32CubeMX]: https://www.st.com/en/development-tools/stm32cubemx.html

[CLion]: https://www.jetbrains.com/clion/

[CLion-Embedded-Development]: https://www.jetbrains.com/help/clion/embedded-development.html
