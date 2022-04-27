#include "rl_handler/rl_handler.hpp"

RL_handler::RL_handler()
    : m_rand_gen(),
      m_learning_rate(0.9),
      m_discount_factor(0.1),
      m_epsilon(0.1),
      m_state{0, 0, 13},
      m_action{0, 0, 0, 5, 3},
      m_model_folder("rl_model/online"),
      state(0, m_state),
      state_next(0, m_state),
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
        ROS_INFO("No model file specified, so skip loading");
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
    }

    //load q_table
    while (!file.eof())
    {
        std::string line;
        std::getline(file, line);
        if (line.find("q_table:") != std::string::npos)
        {
            int8_t state_index = -6;
            std::vector<std::vector<double>> action_vec;

            while (!file.eof())
            {
                std::getline(file, line);

                if (line.find("---") != std::string::npos)
                    break;

                std::vector<double> angular_row;

                if (line[0] == '\n') //update q_table
                {
                    state = rl_state({state_index});
                    for (size_t i = 0; i < action_vec.size(); i++)
                    {
                        for (size_t j = 0; j < angular_row.size(); j++)
                        {
                            action = rl_action(driving_action{int8_t(j - 2), int8_t(i)});
                            policy.update(state, action, action_vec[i][j]);
                        }
                    }
                    state_index++;
                    action_vec.clear();
                    continue;
                }

                std::stringstream ss(line);
                std::string token;
                while (std::getline(ss, token, ','))
                {
                    angular_row.push_back(std::stod(token));
                }
                action_vec.push_back(angular_row);
            }
            break;
        }
    }

    //checking policy logging
    ROS_INFO("Loaded q_table:");

    for (int8_t state_index = -6; state_index <= 6; state_index++)
    {
        for (int8_t angular_index = 0; angular_index <= 2; angular_index++)
        {
            ROS_INFO("state: %d", state_index);
            for (int8_t linear_index = -2; linear_index <= 2; linear_index++)
            {
                ROS_INFO("%s, ", std::to_string(policy.value(relearn::state(semantic_line_state{state_index}), relearn::action(driving_action{angular_index, linear_index}))).c_str());
            }
            ROS_INFO(" ");
        }
        ROS_INFO(" ");
    }

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

    for (int8_t state_index = -6; state_index <= 6; state_index++)
    {
        for (int8_t angular_index = 0; angular_index <= 2; angular_index++)
        {
            for (int8_t linear_index = -2; linear_index <= 2; linear_index++)
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

void RL_handler::update_state()
{
    ROS_INFO("Update state");
    state_next = state;
}

void RL_handler::learn()
{
    ROS_INFO("Learning");
    learner(state, action, state_next, policy, false);

    episode.push_back({state, action});

    ROS_INFO("Episode updated: %lf, %d, %d, %d", episode.back().state.reward(), episode.back().state.trait().offset_discretization, episode.back().action.trait().angular_discretization, action.trait().linear_discretization);
    ROS_INFO("Episode size: %lu", episode.size());
}