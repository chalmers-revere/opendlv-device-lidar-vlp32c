/*
 * opendlv-device-lidar-vlp32c decodes VLP32c data realized for OpenDLV.
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "vlp32c-decoder.hpp"

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

// Calibration data for VLP32c.
const char *VLP32c_XML = R"(
<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>
<!DOCTYPE boost_serialization>
<boost_serialization signature="serialization::archive" version="4">
<DB class_id="0" tracking_level="1" version="0" object_id="_0">
	<distLSB_>0.4</distLSB_>
	<position_ class_id="1" tracking_level="0" version="0">
		<xyz>
			<count>3</count>
			<item>0</item>
			<item>0</item>
			<item>0</item>
		</xyz>
	</position_>
	<orientation_ class_id="2" tracking_level="0" version="0">
		<rpy>
			<count>3</count>
			<item>0</item>
			<item>0</item>
			<item>0</item>
		</rpy>
	</orientation_>
	<colors_ class_id="3" tracking_level="0" version="0">
		<count>64</count>
		<item_version>0</item_version>
		<item class_id="4" tracking_level="0" version="0">
			<rgb>
				<count>3</count>
				<item>0.84018773</item>
				<item>0.39438292</item>
				<item>0.78309923</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.840177</item>
				<item>0.77233541</item>
				<item>0.39436942</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.08879225</item>
				<item>0.8096742</item>
				<item>0.71766233</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.70946825</item>
				<item>0.40906385</item>
				<item>0.82345313</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.5796597</item>
				<item>0.9642939</item>
				<item>0.064805068</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.32542917</item>
				<item>0.69379723</item>
				<item>1</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.96250856</item>
				<item>0.7900511</item>
				<item>0.13542382</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.91102463</item>
				<item>0.34070343</item>
				<item>0.050705731</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.35276073</item>
				<item>0.78958958</item>
				<item>0.90582585</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.21438926</item>
				<item>0.69950408</item>
				<item>0.20341802</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.085374229</item>
				<item>0.94720376</item>
				<item>0.88781565</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.98643476</item>
				<item>0.97503626</item>
				<item>0.034851607</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.7856003</item>
				<item>0.44017774</item>
				<item>0.11511882</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.63264316</item>
				<item>0.34141558</item>
				<item>0.9041099</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.94122225</item>
				<item>0.072755016</item>
				<item>0.85761809</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.95754939</item>
				<item>0.9211719</item>
				<item>0.21469444</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.94947737</item>
				<item>0.55008775</item>
				<item>0.44551766</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.89275962</item>
				<item>0.85084307</item>
				<item>0.01608301</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.91799802</item>
				<item>0.49231708</item>
				<item>0.78014803</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.55109483</item>
				<item>0.75539786</item>
				<item>0.21551843</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.14881226</item>
				<item>0.7401194</item>
				<item>0.53116983</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.93394369</item>
				<item>0.39975587</item>
				<item>0.67215991</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.86502695</item>
				<item>0.56639397</item>
				<item>0.63385367</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.50551611</item>
				<item>0.94326693</item>
				<item>0.48421454</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.087714233</item>
				<item>0.89877903</item>
				<item>0.2479341</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.93935621</item>
				<item>0.80573922</item>
				<item>0.04233218</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.09033341</item>
				<item>0.87373161</item>
				<item>0.9662928</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.13539331</item>
				<item>0.65983063</item>
				<item>0.28789195</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.96635383</item>
				<item>0.28834975</item>
				<item>0.057038225</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.600824</item>
				<item>0.96617073</item>
				<item>0.10191501</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.90740824</item>
				<item>0.95980775</item>
				<item>0.95391774</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.59211904</item>
				<item>0.78711683</item>
				<item>0.86269605</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.028625926</item>
				<item>0.98828107</item>
				<item>0.30778974</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.59491873</item>
				<item>0.35182726</item>
				<item>0.87454033</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.7464866</item>
				<item>0.51860839</item>
				<item>0.8450141</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.32484931</item>
				<item>0.38410011</item>
				<item>0.86562908</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.96064699</item>
				<item>0.052674145</item>
				<item>0.67904174</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.31248951</item>
				<item>0.92227054</item>
				<item>0.48648813</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.054489966</item>
				<item>0.95574886</item>
				<item>0.28729686</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.95848018</item>
				<item>0.93113601</item>
				<item>0.10840009</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.95097274</item>
				<item>0.94096285</item>
				<item>0.32806897</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.9619745</item>
				<item>0.034027617</item>
				<item>0.78579384</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.94944686</item>
				<item>0.82970929</item>
				<item>0.037186235</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.97222859</item>
				<item>0.62723738</item>
				<item>0.065781645</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.92346072</item>
				<item>0.96305794</item>
				<item>0.93403524</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.98861676</item>
				<item>0</item>
				<item>0.76792556</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.26213473</item>
				<item>0.95156789</item>
				<item>0.94323641</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.41365683</item>
				<item>0.97004652</item>
				<item>0.078889146</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.85752654</item>
				<item>0.1990692</item>
				<item>0.22206454</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.053299762</item>
				<item>0.93763638</item>
				<item>0.99444574</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.8943212</item>
				<item>0.62864298</item>
				<item>0.10409617</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.76058036</item>
				<item>0.048754983</item>
				<item>0.91462308</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.83480585</item>
				<item>0.70650798</item>
				<item>0.51032275</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.89080644</item>
				<item>0.12384222</item>
				<item>0.26840618</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.32953385</item>
				<item>0.81698328</item>
				<item>0.51252002</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.39465934</item>
				<item>0.75048447</item>
				<item>0.75953305</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.92498666</item>
				<item>0.94149691</item>
				<item>0.12957962</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.91115069</item>
				<item>0.52652103</item>
				<item>0.73026121</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.76655</item>
				<item>0.93766737</item>
				<item>0.031084489</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.12669764</item>
				<item>0.85318863</item>
				<item>0.33537352</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.9853164</item>
				<item>0.26496059</item>
				<item>0.13821837</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.84317338</item>
				<item>0.17844476</item>
				<item>0.94212615</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.69955063</item>
				<item>0.59048992</item>
				<item>0.74391818</item>
			</rgb>
		</item>
		<item>
			<rgb>
				<count>3</count>
				<item>0.67139697</item>
				<item>0.87832457</item>
				<item>0.69022661</item>
			</rgb>
		</item>
	</colors_>
	<enabled_>
		<count>64</count>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>1</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
	</enabled_>
	<intensity_>
		<count>64</count>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
	</intensity_>
	<minIntensity_>
		<count>64</count>
		<item_version>0</item_version>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
		<item>0</item>
	</minIntensity_>
	<maxIntensity_>
		<count>64</count>
		<item_version>0</item_version>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
		<item>255</item>
	</maxIntensity_>
	<points_ class_id="7" tracking_level="0" version="0">
		<count>64</count>
		<item_version>1</item_version>
		<item class_id="8" tracking_level="0" version="1">
			<px class_id="9" tracking_level="1" version="1" object_id="_1">
				<id_>0</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>-25.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_2">
				<id_>1</id_>
				<rotCorrection_>4.200</rotCorrection_>
				<vertCorrection_>-1.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_3">
				<id_>2</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>-1.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_4">
				<id_>3</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>-15.639</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_5">
				<id_>4</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>-11.310</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_6">
				<id_>5</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>0.000 </vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_7">
				<id_>6</id_>
				<rotCorrection_>-4.200</rotCorrection_>
				<vertCorrection_>-0.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_8">
				<id_>7</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>-8.843</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_9">
				<id_>8</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>-7.254</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_10">
				<id_>9</id_>
				<rotCorrection_>4.200</rotCorrection_>
				<vertCorrection_>0.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_11">
				<id_>10</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>-0.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_12">
				<id_>11</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>-6.148</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_13">
				<id_>12</id_>
				<rotCorrection_>-4.200</rotCorrection_>
				<vertCorrection_>-5.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_14">
				<id_>13</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>1.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_15">
				<id_>14</id_>
				<rotCorrection_>-4.200</rotCorrection_>
				<vertCorrection_>0.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_16">
				<id_>15</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>-4.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_17">
				<id_>16</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>-4.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_18">
				<id_>17</id_>
				<rotCorrection_>4.200</rotCorrection_>
				<vertCorrection_>1.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_19">
				<id_>18</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>1.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_20">
				<id_>19</id_>
				<rotCorrection_>4.200</rotCorrection_>
				<vertCorrection_>-3.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_21">
				<id_>20</id_>
				<rotCorrection_>-4.200</rotCorrection_>
				<vertCorrection_>-3.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_22">
				<id_>21</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>3.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_23">
				<id_>22</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>2.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_24">
				<id_>23</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>-2.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_25">
				<id_>24</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>-3.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_26">
				<id_>25</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>7.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_27">
				<id_>26</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>4.667</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_28">
				<id_>27</id_>
				<rotCorrection_>4.200</rotCorrection_>
				<vertCorrection_>-2.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_29">
				<id_>28</id_>
				<rotCorrection_>-4.200</rotCorrection_>
				<vertCorrection_>-2.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_30">
				<id_>29</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>15.000</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_31">
				<id_>30</id_>
				<rotCorrection_>-1.400</rotCorrection_>
				<vertCorrection_>10.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_32">
				<id_>31</id_>
				<rotCorrection_>1.400</rotCorrection_>
				<vertCorrection_>-1.333</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_33">
				<id_>32</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_34">
				<id_>33</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_35">
				<id_>34</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_36">
				<id_>35</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_37">
				<id_>36</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_38">
				<id_>37</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_39">
				<id_>38</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_40">
				<id_>39</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_41">
				<id_>40</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_42">
				<id_>41</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_43">
				<id_>42</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_44">
				<id_>43</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_45">
				<id_>44</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_46">
				<id_>45</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_47">
				<id_>46</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_48">
				<id_>47</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_49">
				<id_>48</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_50">
				<id_>49</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_51">
				<id_>50</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_52">
				<id_>51</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_53">
				<id_>52</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_54">
				<id_>53</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_55">
				<id_>54</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_56">
				<id_>55</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_57">
				<id_>56</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_58">
				<id_>57</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_59">
				<id_>58</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_60">
				<id_>59</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_61">
				<id_>60</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_62">
				<id_>61</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_63">
				<id_>62</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
		<item>
			<px class_id_reference="9" object_id="_64">
				<id_>63</id_>
				<rotCorrection_>0</rotCorrection_>
				<vertCorrection_>0</vertCorrection_>
				<distCorrection_>0</distCorrection_>
				<distCorrectionX_>0</distCorrectionX_>
				<distCorrectionY_>0</distCorrectionY_>
				<vertOffsetCorrection_>0</vertOffsetCorrection_>
				<horizOffsetCorrection_>0</horizOffsetCorrection_>
				<focalDistance_>0</focalDistance_>
				<focalSlope_>0</focalSlope_>
			</px>
		</item>
	</points_>
</DB>
</boost_serialization>
)";

////////////////////////////////////////////////////////////////////////////////

VLP32cDecoder::VLP32cDecoder(int32_t intensity) noexcept
    : m_intensityBitsMSB(intensity) {
    setupCalibration();
    index32sensorIDs();
}

void VLP32cDecoder::setupCalibration() noexcept {
    // A VLP-32E has 32 channels/sensors. Each sensor has a specific vertical
    // angle, which can be read from m_verticalAngle[sensor ID] as specified
    // in the calibration file.
    const std::string CALIBRATION(VLP32c_XML);
    std::stringstream sstr(CALIBRATION);

    bool found{false};
    uint8_t counter{0}; // Corresponds to the index of the vertical angle of each beam.
    std::string line;
    while (getline(sstr, line) && (32 > counter)) {
        std::string tmp; // Strip whitespaces from the beginning.
        for (uint8_t i = 0; i < line.length(); i++) {
            if (!((line[i] == '\t' || line[i] == ' ') && tmp.size() == 0)) {
                if (line[i] == '<') {
                    if (found) {
                        m_verticalAngle[counter] = static_cast<float>(std::atof(tmp.c_str()));
                        counter++;
                        found = false;
                        continue;
                    }
                    tmp += line[i];
                }
                else {
                    tmp += line[i];
                }
            }

            if (tmp == "<vertCorrection_>") {
                found = true;
                tmp = "";
            }
        }
    }
}

void VLP32cDecoder::index32sensorIDs() noexcept {
    // Distance values for each 32 sensors with the same azimuth are ordered
    // based on vertical angle, from -25.0 to 15 degrees. The data is sent
    // in three chunks: verticalAngles12 = [ -25.0, -15.639, -7.254, -4.667, 
    // -3.333, -2.333, -1.333, -0.333, 0.667, 1.667, 4.667, 15.0 ];
    // verticalAngles11 = [ -11.31, -8.843, -5.333, -3.667, -2.667, -1.667, 
    // -0.667, 0.333, 1.333, 3.333, 10.333 ]; verticalAngles9 = [ -6.148, -4.0, 
    // -3.0, -2.0, -1.0, 0.0, 1.0, 2.333, 7.0 ];

    std::array<float, 32> orderedVerticalAngle;
    for (uint8_t i = 0; i < 32; i++) {
        m_32Sensors[i] = 0;
        orderedVerticalAngle[i] = m_verticalAngle[i];
    }

    // Order the vertical angles of 32 sensor IDs with increasing value.
    for (uint8_t i = 0; i < 32; i++) {
        for (uint8_t j = i; j < 32; j++) {
            if (orderedVerticalAngle[j] < orderedVerticalAngle[i]) {
                std::swap(orderedVerticalAngle[j], orderedVerticalAngle[i]);
            }
        }
    }

    // Find the sensor IDs in the odered list of vertical angles.
    for (uint8_t i = 0; i < 32; i++) {
        for (uint8_t j = 0; j < 32; j++) {
            if (std::abs(orderedVerticalAngle[i] - m_verticalAngle[j]) < 0.1f) {
                m_sensorOrderIndex[i] = j;
                break;
            }
        }
    }
}

std::vector<opendlv::proxy::PointCloudReading> VLP32cDecoder::decode(const std::string &data) noexcept {
    std::vector<opendlv::proxy::PointCloudReading> retVal;

    if (1206 == data.size()) {
        // Decode VLP-32 data.
        uint32_t position = 0; // Position specifies the starting position to read from the 1206 bytes.

        // The data of a VLP-32 packet consists of 12 blocks with 100 bytes each. Decode each block separately.
        uint8_t firstByte{0}, secondByte{0}, thirdByte{0}; // Two bytes for distance and one byte for intensity.
        uint16_t dataValue{0};
        for (uint8_t blockID{0}; blockID < 12; blockID++) {
            // Skip the flag: 0xEEFF for upper block or 0xDDFF for lower block (2 bytes).
            position += 2;

            // Decode azimuth information: 2 bytes. Swap the two bytes, change to decimal, and divide it by 100.
            firstByte = static_cast<uint8_t>(data.at(position));
            secondByte = static_cast<uint8_t>(data.at(position + 1));
            dataValue = be16toh(firstByte * 256 + secondByte);
            m_currentAzimuth = static_cast<float>(dataValue/100.0f);

            if (m_currentAzimuth < m_previousAzimuth) {
                // Return data from complete sweep.
                opendlv::proxy::PointCloudReading pointCloudPart1;
                pointCloudPart1.startAzimuth(m_startAzimuth)
                               .endAzimuth(m_previousAzimuth)
                               .entriesPerAzimuth(12)
                               .distances(m_distanceStringStreamPart1.str())
                               .numberOfBitsForIntensity(m_intensityBitsMSB)
                               .typeOfVerticalAngularLayout(1); // Indicate VLP32c.

                opendlv::proxy::PointCloudReading pointCloudPart2;
                pointCloudPart2.startAzimuth(m_startAzimuth)
                               .endAzimuth(m_previousAzimuth)
                               .entriesPerAzimuth(11)
                               .distances(m_distanceStringStreamPart2.str())
                               .numberOfBitsForIntensity(m_intensityBitsMSB)
                               .typeOfVerticalAngularLayout(1); // Indicate VLP32c.

                opendlv::proxy::PointCloudReading pointCloudPart3;
                pointCloudPart3.startAzimuth(m_startAzimuth)
                               .endAzimuth(m_previousAzimuth)
                               .entriesPerAzimuth(9)
                               .distances(m_distanceStringStreamPart3.str())
                               .numberOfBitsForIntensity(m_intensityBitsMSB)
                               .typeOfVerticalAngularLayout(1); // Indicate VLP32c.

                retVal.push_back(pointCloudPart1);
                retVal.push_back(pointCloudPart2);
                retVal.push_back(pointCloudPart3);

                m_pointIndexCPC = 0;
                m_startAzimuth = m_currentAzimuth;
                m_distanceStringStreamPart1.str("");
                m_distanceStringStreamPart2.str("");
                m_distanceStringStreamPart3.str("");
            }

            m_previousAzimuth = m_currentAzimuth;
            position += 2;

            // Only decode the data if the maximum number of points of the
            // current frame has not been reached.
            if (m_pointIndexCPC < MAX_POINT_SIZE) {
                // Decode distance information and intensity of each beam/channel in a block.
                for (uint8_t counter{0}; counter < 32; counter++) {
                    uint8_t sensorID = counter;

                    // Decode distance: 2 bytes. Swap bytes.
                    firstByte = (uint8_t)(data.at(position));
                    secondByte = (uint8_t)(data.at(position + 1));
                    thirdByte = (uint8_t)(data.at(position + 2)); // Original intensity value.

                    if (m_pointIndexCPC < MAX_POINT_SIZE) {
                        // Store distance with resolution 0.02m in an array of uint16_t type.
                        m_32Sensors[sensorID] = be16toh(firstByte * 256 + secondByte);

                        // TODO: Always in cm encoding for now.
                        if (m_distanceEncoding == 0) {
                            m_32Sensors[sensorID] = static_cast<uint16_t>(m_32Sensors[sensorID]/5.0f); // Store distance with resolution 1cm instead
                        }

                        if (m_intensityBitsMSB > 0) {
                            uint16_t distanceWithIntensity{m_32Sensors[sensorID]};
                            const uint16_t MASK = 0xFFFF >> m_intensityBitsMSB;
                            distanceWithIntensity &= MASK;
                            const uint16_t INTENSITY = thirdByte >> (8 - m_intensityBitsMSB);
                            distanceWithIntensity += (INTENSITY << (16 - m_intensityBitsMSB) );
                            m_32Sensors[sensorID] = distanceWithIntensity;
                        }

                        if (sensorID == 31) {
                            for (uint8_t index{0}; index < 32; index++) {
                                m_32Sensors[m_sensorOrderIndex[index]] = htobe16(m_32Sensors[m_sensorOrderIndex[index]]);
                                if (index == 0 || (index % 3 == 1)) {
                                    // Layer 0, 1, 4, 7..., i.e., in addition to Layer 0, every 3rd layer from Layer 1 and resulting in 12 layers
                                    m_distanceStringStreamPart1.write(reinterpret_cast<char*>(&m_32Sensors[m_sensorOrderIndex[index]]), 2);
                                }
                                else if (index == 2 || (index % 3 == 0)) {
                                    // Layer 2, 3, 6, 9..., i.e., in addition to Layer 2, every 3rd layer from Layer 3 and resulting in 11 layers
                                    m_distanceStringStreamPart2.write(reinterpret_cast<char*>(&m_32Sensors[m_sensorOrderIndex[index]]), 2);
                                }
                                else {
                                    // Layer 5, 8, 11..., i.e., every 3rd layer from Layer 5 and resulting in 9 layers
                                    m_distanceStringStreamPart3.write(reinterpret_cast<char*>(&m_32Sensors[m_sensorOrderIndex[index]]), 2);
                                }
                            }
                        }
                        m_pointIndexCPC++;
                    }
                    position += 3;

                    if (m_pointIndexCPC >= MAX_POINT_SIZE) {
                        position += 3 * (31 - counter); // Discard the points of the current frame when the preallocated shared memory is full; move the position to be read in the 1206 bytes
                        std::cerr << "[lidar-vlp32c] More than 70,000 points?" << std::endl; 
                        break;
                    }
                }
            }
            else {
                position += 96; // 32*3 bytes, skipping one block.
            }
        }

        // TODO: Decode 4 bytes GPS time stamp.

        // Skip 2 blank bytes.
    }

    return retVal;
}

