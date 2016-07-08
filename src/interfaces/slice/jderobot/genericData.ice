/*
 *
 *  Copyright (C) 1997-2015 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Author : Francisco Rivas <gchanfr@gmail.com>
 *
 */

#ifndef GENERICDATA_ICE
#define GENERICDATA_ICE

#include <jderobot/common.ice>


module jderobot{  


  /**
   *  Static description of the image source.
   */
  class GenericData 
  {
    string name; /**< %The name of the file  */
    string format;/**< % The format of the file */
    ByteSeq data; /**< % The file data itself*/
  };
  

  interface genericDataProvider
  {
    /** 
     * Returns the generic data class
     */
    idempotent GenericData getGenericData();

  };

}; //module

#endif //GENERICDATA_ICE
