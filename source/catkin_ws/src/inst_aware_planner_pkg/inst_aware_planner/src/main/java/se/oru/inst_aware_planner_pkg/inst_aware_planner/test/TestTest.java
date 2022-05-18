package se.oru.inst_aware_planner_pkg.inst_aware_planner.test;

public class TestTest {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		isRequirementAct("blablabla::SayWord", "SayWord");
		System.out.println(getRobotNameFromRequirement("robot1::SayWord"));
		
		isActGoal("Goall_blabla");
		isActGoal("Goal_blabla");
		

	}
	
	public static Boolean isRequirementAct(String requirement, String act){
		Integer colIndex = requirement.lastIndexOf("::");
		String reqAct = requirement.substring(colIndex+2);
		
		if (reqAct.equals(act))
			return true;
		
		return false;
	}
	
	public static String getRobotNameFromRequirement(String requirement)	{
		
		Integer colIndex = requirement.lastIndexOf("::");
		String agent = requirement.substring(0, colIndex);
		
		return agent;
	}
	
	private static Boolean isActGoal(String actName){
		
		Integer colIndex = actName.lastIndexOf("Goal_");
		if (colIndex == 0) {
			return true;
		}
		
		return false;
	}

}
