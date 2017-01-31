

public class Test {

	static boolean cont = true;
	public static void main(String args[])
	{
		MyThread mt = new MyThread();
		mt.start();
		while(cont)
		{
		try{Thread.sleep(1);}catch(Exception e){};
		}
		System.out.println("Hello");
	}
	
	
}

class MyThread extends Thread
{
   public void run ()
   {
      for (int count = 1, row = 1; row < 20; row++, count++)
      {
           for (int i = 0; i < count; i++)
                System.out.print ('*');
           System.out.print ('\n');
           if(count == 5)
           {
        	   Test.cont = false;
        	   break;
           }
      }
   }
}