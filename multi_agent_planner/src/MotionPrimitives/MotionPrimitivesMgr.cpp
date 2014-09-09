#include <multi_agent_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <multi_agent_planner/Constants.h>

using namespace multi_agent_planner;

MotionPrimitivesMgr::MotionPrimitivesMgr(std::shared_ptr<GoalState>& goal)
    : m_goal(goal) { }

void MotionPrimitivesMgr::setMprimParams(const MotionPrimitiveParams& params) {
    m_params = params;
}


/*! \brief loads all mprims from configuration. also sets up amps. note that
 * these are not necessarily the exact mprims used during search, because
 * there's a user parameter that selects which mprims to actually use. 
 */
bool MotionPrimitivesMgr::loadMPrims(){
    // generate the nav mprims
    MPrimList nav_mprims;
    loadNavPrims(nav_mprims);

    m_all_mprims = nav_mprims;

    for (auto& mprim: m_all_mprims){    
        mprim->print();
    }

    return true;
}

void MotionPrimitivesMgr::loadNavPrims(MPrimList& nav_mprims) {
    const int NUM_DIRS = 17;
    std::vector<int> dx_(NUM_DIRS, 0), dy_(NUM_DIRS, 0);
    std::vector<int> dx0intersects_(NUM_DIRS, 0), dy0intersects_(NUM_DIRS, 0);
    std::vector<int> dx1intersects_(NUM_DIRS, 0), dy1intersects_(NUM_DIRS, 0);
    std::vector<int> dxy_distance_mm_(NUM_DIRS, 0);

    dx_[0] = 1;
    dy_[0] = 1;
    dx0intersects_[0] = -1;
    dy0intersects_[0] = -1;
    dx_[1] = 1;
    dy_[1] = 0;
    dx0intersects_[1] = -1;
    dy0intersects_[1] = -1;
    dx_[2] = 1;
    dy_[2] = -1;
    dx0intersects_[2] = -1;
    dy0intersects_[2] = -1;
    dx_[3] = 0;
    dy_[3] = 1;
    dx0intersects_[3] = -1;
    dy0intersects_[3] = -1;
    dx_[4] = 0;
    dy_[4] = -1;
    dx0intersects_[4] = -1;
    dy0intersects_[4] = -1;
    dx_[5] = -1;
    dy_[5] = 1;
    dx0intersects_[5] = -1;
    dy0intersects_[5] = -1;
    dx_[6] = -1;
    dy_[6] = 0;
    dx0intersects_[6] = -1;
    dy0intersects_[6] = -1;
    dx_[7] = -1;
    dy_[7] = -1;
    dx0intersects_[7] = -1;
    dy0intersects_[7] = -1;
    dx_[8] = 0;
    dy_[8] = 0;
    dx0intersects_[8] = -1;
    dy0intersects_[8] = -1;


    dx_[9] = 2; dy_[9] = 1;
    dx0intersects_[9] = 1; dy0intersects_[9] = 0; dx1intersects_[9] = 1; dy1intersects_[9] = 1;
    dx_[10] = 1; dy_[10] = 2;
    dx0intersects_[10] = 0; dy0intersects_[10] = 1; dx1intersects_[10] = 1; dy1intersects_[10] = 1;
    dx_[11] = -1; dy_[10] = 2;
    dx0intersects_[11] = 0; dy0intersects_[11] = 1; dx1intersects_[11] = -1; dy1intersects_[11] = 1;
    dx_[12] = -2; dy_[12] = 1;
    dx0intersects_[12] = -1; dy0intersects_[12] = 0; dx1intersects_[12] = -1; dy1intersects_[12] = 1;
    dx_[13] = -2; dy_[13] = -1;
    dx0intersects_[13] = -1; dy0intersects_[13] = 0; dx1intersects_[13] = -1; dy1intersects_[13] = -1;
    dx_[14] = -1; dy_[14] = -2;
    dx0intersects_[14] = 0; dy0intersects_[14] = -1; dx1intersects_[14] = -1; dy1intersects_[14] = -1;
    dx_[15] = 1; dy_[15] = -2;
    dx0intersects_[15] = 0; dy0intersects_[15] = -1; dx1intersects_[15] = 1; dy1intersects_[15] = -1;
    dx_[16] = 2; dy_[16] = -1;
    dx0intersects_[16] = 1; dy0intersects_[16] = 0; dx1intersects_[16] = 1; dy1intersects_[16] = -1;

    // There are actually 17 motion primitives because we want one to wait in
    // place while the others "catch-up"
    // TODO: Introduce wait-in-place primitive here.

    //compute costs
    for (int dind = 0; dind < NUM_DIRS; dind++) {
        if (dx_[dind] != 0 && dy_[dind] != 0) {
            if (dind <= 8)
                //the cost of a diagonal move in millimeters
                dxy_distance_mm_[dind] = static_cast<int>(m_params.env_resolution * 1414); 
            else
                //the cost of a move to 1,2 or 2,1 or so on in millimeters
                dxy_distance_mm_[dind] = static_cast<int>(m_params.env_resolution * 2236); 
        } else {
            // we set the cost of no move to be the same as a horizontal move.
            // zero-cost actions are not nice.
            dxy_distance_mm_[dind] = static_cast<int>(m_params.env_resolution * 1000); //the cost of a horizontal move in millimeters
        }
    }

    // create the MPrims themselves
    for (int i = 0; i < NUM_DIRS; i++) {
        // make end coords
        GraphStateMotion end_coords(ROBOT_DOF, 0);
        end_coords[RobotStateElement::X] = dx_[i];
        end_coords[RobotStateElement::Y] = dy_[i];

        IntermSteps interm_steps;
        // make intermsteps (if any)
        if (i > 8) {
            interm_steps.resize(2);
            interm_steps[0] = std::vector<double>{
                                    static_cast<double>(dx0intersects_[i]),
                                    static_cast<double>(dy0intersects_[i])
                                };
            interm_steps[1] = std::vector<double>{
                                    static_cast<double>(dx1intersects_[i]),
                                    static_cast<double>(dy1intersects_[i])
                                };
        }

        // fill up the primitive
        NavMotionPrimitivePtr prim = std::make_shared<NavMotionPrimitive>();
        prim->setID(i);
        prim->setEndCoord(end_coords);
        prim->setIntermSteps(interm_steps);
        prim->setBaseCost(dxy_distance_mm_[i]);
        nav_mprims.push_back(prim);
    }
}