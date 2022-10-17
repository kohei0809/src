package tools;

import java.util.List;
import java.util.Map;

public class BinarySearchDecending{
    //降順の二分探索

    public static int binarySearchDecendingInteger(List<Map.Entry<Integer, Integer>> array, int count){
        //同じ値の基準値
        int sameValue = array.get(count).getValue();
        int same = count;
        int small = array.size() - 1;
        int check = (same + small) / 2;

        do{
            if(array.get(check).getValue() == sameValue){
                same = check;
            }
            else if(array.get(check).getValue() < sameValue){
                small = check;
            }

            check = (same + small) / 2;
        
        } while(!(check == same || check == small));
        
        return same + 1;
    }

    public static int binarySearchDecendingDouble(List<Map.Entry<Integer, Double>> array, int count){
         //同じ値の基準値
         double sameValue = array.get(count).getValue();
         int same = count;
         int small = array.size() - 1;
         int check = (same + small) / 2;
 
         do{
            if(array.get(check).getValue() == sameValue){
                same = check;
            }
            else if(array.get(check).getValue() < sameValue){
                small = check;
            }
 
            check = (same + small) / 2;
         
        } while(!(check == same || check == small));
         
        return same + 1;
    }
}