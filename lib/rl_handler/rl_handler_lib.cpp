#include "rl_handler/rl_handler.hpp"

RL_handler::RL_handler()
    : m_rand_gen(),
      m_learning_rate(0.9),
      m_discount_factor(0.1),
      m_epsilon(0.1),
      m_state{0, 0, 7},
      m_action{0, 0, 0, 5, 3},
      m_model_folder("rl_model/online"),
      state(0.0, m_state),
      state_next(0.0, m_state),
      action{m_action},
      policy(),
      episode(),
      learner{m_learning_rate, m_discount_factor}
{
    ROS_INFO("RL_handler constructed");
}

RL_handler::~RL_handler()
{
    ROS_INFO("RL_handler destructed");
}

void RL_handler::init()
{
    init_rand_generator();

    state = rl_state(0, m_state);
    state_next = rl_state(0, m_state);
    action = rl_action{m_action};

    episode.clear();
    policy.clear();
}

void RL_handler::init_rand_generator()
{
    m_rand_gen = std::mt19937(static_cast<std::size_t>(std::chrono::high_resolution_clock::now()
                                                           .time_since_epoch()
                                                           .count()));
}

void RL_handler::load_model(const std::string &filename)
{
    ROS_INFO("Loading model: %s", filename.c_str());

    if (filename == "")
    {
        ROS_WARN("No model file specified, so skip loading");
        return;
    }

    std::filesystem::create_directories(m_model_folder);
    std::filesystem::path model_path = m_model_folder / filename;

    std::ifstream file(model_path.c_str(), std::ios::in);
    if (!file)
    {
        ROS_ERROR("Failed to open file: %s", model_path.c_str());
        return;
    }

    //load model parameters
    while (!file.eof())
    {
        std::string line;
        std::getline(file, line);
        if (line.find("learning_rate: ") != std::string::npos)
        {
            std::stringstream ss(line);
            std::string token;
            std::getline(ss, token, ':');
            std::getline(ss, token);
            m_learning_rate = std::stod(token); //std::stod actually ignores string space
            ROS_INFO("Learning rate: %f", m_learning_rate);
        }
        if (line.find("discount_factor: ") != std::string::npos)
        {
            std::stringstream ss(line);
            std::string token;
            std::getline(ss, token, ':');
            std::getline(ss, token);
            m_discount_factor = std::stod(token);
            ROS_INFO("Discount factor: %f", m_discount_factor);
        }
        if (line.find("epsilon: ") != std::string::npos)
        {
            std::stringstream ss(line);
            std::string token;
            std::getline(ss, token, ':');
            std::getline(ss, token);
            m_epsilon = std::stod(token);
            ROS_INFO("Epsilon: %f", m_epsilon);
        }
        if (line.find("q_table:") != std::string::npos)
        {
            int8_t state_index = -3;
            std::vector<std::vector<double>> action_vec;
            std::vector<double> angular_row;

            while (!file.eof())
            {
                std::getline(file, line);
                ROS_INFO("file line: %s", line.c_str());

                if (line.find("---") != std::string::npos)
                    break;

                if (line == "") //update q_table
                {
                    auto state_temp = rl_state(semantic_line_state{state_index});
                    ROS_INFO("state index:%d", state_index);
                    for (size_t i = 0; i < 3; i++) //size: 3, linear: 0 ~ 2
                    {
                        for (size_t j = 0; j < 5; j++) //size: 5, angular: -2 ~ 2
                        {
                            auto action_temp = rl_action(driving_action{int8_t(int(j) - 2), int8_t(i)});
                            policy.update(state_temp, action_temp, action_vec[i][j]);
                        }
                        ROS_INFO("action element: %f, %f, %f, %f, %f", action_vec[i][0], action_vec[i][1],
                                 action_vec[i][2], action_vec[i][3], action_vec[i][4]);
                    }
                    state_index++;
                    angular_row.clear();
                    action_vec.clear();
                    continue;
                }

                std::stringstream ss(line);
                std::string token;
                while (std::getline(ss, token, ',') && token != " ")
                {
                    angular_row.push_back(std::stod(token));
                }
                action_vec.push_back(angular_row);
                angular_row.clear();

                // for(auto &inner_vec : action_vec)
                // {
                //     for(auto &element : inner_vec)
                //     {
                //         ROS_INFO("element: %f", element);
                //     }
                // }
            }
            break;
        }
    }

    //checking policy logging
    ROS_INFO("Q_table checking:");

    for (int8_t state_index = -3; state_index <= 3; state_index++)
    {
        for (int8_t linear_index = 0; linear_index <= 2; linear_index++)
        {
            ROS_INFO("state: %d", state_index);
            for (int8_t angular_index = -2; angular_index <= 2; angular_index++)
            {
                ROS_INFO("%s, ", std::to_string(policy.value(relearn::state(semantic_line_state{state_index}), relearn::action(driving_action{angular_index, linear_index}))).c_str());
            }
            ROS_INFO(" ");
        }
        ROS_INFO(" ");
    }
    ROS_INFO("End of Q_table checking");

    file.close();
}

void RL_handler::save_model(const std::string &filename)
{
    ROS_INFO("Saving model: %s", filename.c_str());

    std::filesystem::create_directories(m_model_folder);
    std::filesystem::path model_path = m_model_folder / filename;

    std::ofstream file(model_path.c_str());

    // if (!file)
    // {
    //     ROS_INFO("Failed to open file: %s", model_path.c_str());
    //     ROS_INFO("Now to create a file");

    //     std::ofstream file(model_path.c_str(), std::ios::out);
    // }

    //save model parameter
    file << "---"
         << "\n";

    file << "learning_rate: " << m_learning_rate << "\n";
    file << "discount_factor: " << m_discount_factor << "\n";
    file << "epsilon: " << m_epsilon << "\n";

    file << "---"
         << "\n";

    file << "episode: (state: (reward, offset), action: (angular, linear)) "
         << "\n";

    //save episode
    for (auto &e : this->episode)
    {
        file << "state: (" << std::to_string(e.state.reward()) << ", " << std::to_string(e.state.trait().offset_discretization) << "), ";
        file << "action: (" << std::to_string(e.action.trait().angular_discretization) << ", " << std::to_string(e.action.trait().linear_discretization) << ") "
             << "\n";
        //note: the type should explicitly be specified, since file output is binary by default
    }

    file << "---"
         << "\n";

    file << "q_table: "
         << "\n";

    for (int8_t state_index = -3; state_index <= 3; state_index++)
    {
        for (int8_t linear_index = 0; linear_index <= 2; linear_index++)
        {
            for (int8_t angular_index = -2; angular_index <= 2; angular_index++)
            {
                file << std::to_string(policy.value(relearn::state(semantic_line_state{state_index}), relearn::action(driving_action{angular_index, linear_index}))) << ", ";
            }
            file << "\n";
        }
        file << "\n";
    }

    file << "---";

    file.close();
}

std::string RL_handler::get_filename_by_cur_time()
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string filename = ss.str();

    return filename;
}

std::string RL_handler::get_recent_filename()
{
    //TODO: implement regex to verify the file name
    std::filesystem::create_directories(m_model_folder);
    std::filesystem::path model_path = m_model_folder;
    std::vector<std::string> filenames;
    for (auto &p : std::filesystem::directory_iterator(model_path))
    {
        filenames.push_back(p.path().filename().string());
    }
    if (filenames.empty())
    {
        ROS_INFO("No model found");
        return "";
    }
    std::sort(filenames.begin(), filenames.end());
    std::string filename = filenames.back();
    return filename;
}

void RL_handler::get_action_epsilon() //epsilon greedy
{
    std::uniform_real_distribution<double> rand_num(0.0, 1.0);
    if (rand_num(m_rand_gen) > m_epsilon)
        best_action(); //selected from the policy
    else
        rand_action();
}

void RL_handler::rand_action()
{
    std::uniform_int_distribution<int8_t> angular_gen(-2, 2);
    std::uniform_int_distribution<int8_t> linear_gen(0, 3);

    auto angular = angular_gen(m_rand_gen);
    auto linear = linear_gen(m_rand_gen);

    action = rl_action(driving_action{angular, linear});

    ROS_INFO("Random action: %d, %d", action.trait().angular_discretization, action.trait().linear_discretization);
}

void RL_handler::best_action()
{
    auto action_ptr = policy.best_action(state);
    if (action_ptr == nullptr)
    {
        ROS_INFO("No action found, switching to random action");
        action = rl_action(driving_action{0, 0});
        rand_action();
        return;
    }
    ROS_INFO("Best action: %d, %d", action_ptr->trait().angular_discretization, action_ptr->trait().linear_discretization);

    action = *(action_ptr);
}

void RL_handler::set_state(double reward, semantic_line_state &state_trait)
{
    state = relearn::state(reward, state_trait);
    ROS_INFO("got state: %f, %d", state.reward(), state.trait().offset_discretization);
}

void RL_handler::set_next_state(double reward, semantic_line_state &next_state)
{
    state_next = rl_state(reward, next_state);
    ROS_INFO("got next state: %f, %d", state_next.reward(), state_next.trait().offset_discretization);
}

void RL_handler::update_state()
{
    ROS_INFO("Update state");
    state = state_next;
}

void RL_handler::learn()
{
    ROS_INFO("Learning");
    learner(state, action, state_next, policy, false);

    episode.push_back({state, action});

    ROS_INFO("Episode updated: %f, %d, %d, %d", episode.back().state.reward(), episode.back().state.trait().offset_discretization, episode.back().action.trait().angular_discretization, action.trait().linear_discretization);
    ROS_INFO("Episode size: %lu", episode.size());
}