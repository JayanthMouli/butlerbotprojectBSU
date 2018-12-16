<!--                         JAYANTH MOULI
                            BOISE HIGH SCHOOL
                            BSU INTERN 2018-19
                            
.^o ~\
Y /'~) }      _____
l/  / /    ,-~     ~~--.,_
   ( (    /  ~-._         ^.
    \ "--'--.    "-._       \
     "-.________     ~--.,__ ^.
               "~r-.,___.-'-. ^.
                YI    \      ~-.\
                ||     \        `\
                ||     //
                ||    //
                ()   //
                ||  //     
                || ( c
   ___._ __  ___I|__`--__._ __  _
 "~     ~  "~   ::  ~~"    ~  ~~
                ::
                .:
                 .
-->


<?php
	$con = mysqli_connect("db4free.net", "ece218remote", "butlercall", "butlerbot_destdb");
	if(mysqli_connect_errno($con)) {
	echo "Failed to connect";
	}
	$sql = "SELECT * FROM dest_values";
	$retval = mysqli_query($con, $sql);
	if(! $retval ) {
      die('Could not get data: ' . mysql_error());
    }
    $results = mysqli_fetch_all($retval);
    $colsize = sizeof($results);
    
    $final = $results[$colsize-1][0];
    echo $final;

    ?>
