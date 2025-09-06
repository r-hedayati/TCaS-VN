//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
simple Txc10
{
    parameters:
        @display("i=block/routing");

    gates:
        input in[]; //dar sorati ke bekhahim chand port vorodi dashte bashim. 
        output out[];
        // ya estefade az gate inout
      //  inout gate[];  // declare two way connections
}

network Tictoc10
{
    types: // faqat dakhel hamin network qabel estefade ast.
       channel Channel extends ned.DelayChannel {
	   delay = 100ms;
	        }
    submodules:
        //adad dakhel araye neshan dahande tedad gate haye tarif shode ast.
        tic[6]: Txc10 {
            parameters:
                @display("i=,cyan");
        }
    connections:
            tic[0].out++ --> Channel --> tic[1].in++;
	        tic[0].in++ <-- Channel <-- tic[1].out++;
	
	        tic[1].out++ --> Channel --> tic[2].in++;
	        tic[1].in++ <-- Channel <-- tic[2].out++;
	
	        tic[1].out++ --> Channel --> tic[4].in++;
	        tic[1].in++ <-- Channel <-- tic[4].out++;
	
	        tic[3].out++ --> Channel --> tic[4].in++;
	        tic[3].in++ <-- Channel <-- tic[4].out++;
	
	        tic[4].out++ --> Channel --> tic[5].in++;
	        tic[4].in++ <-- Channel <-- tic[5].out++;
	        
	        // dar sorati k gate inout tarif konim connection haye ma in shekli mishe
	        
	        //tic[0].gate++ <--> Channel <--> tic[1].gate++;
	        //tic[1].gate++ <--> Channel <--> tic[2].gate++;
	        //tic[1].gate++ <--> Channel <--> tic[4].gate++;
	        //tic[3].gate++ <--> Channel <--> tic[4].gate++;
	        //tic[4].gate++ <--> Channel <--> tic[5].gate++;

}
